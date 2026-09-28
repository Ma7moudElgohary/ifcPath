#include "IFCPathSubsystem.h"

#include "Algo/Reverse.h"

namespace
{
    constexpr double HierarchySnapDistanceCm = 300.0;
    constexpr double HierarchyCellPlaneToleranceCm = 200.0;
    constexpr double HierarchyPointToleranceCm = 0.01;

    bool IsVerticalKind(const FString& Kind)
    {
        return Kind == TEXT("stair")
            || Kind == TEXT("ramp")
            || Kind == TEXT("elevator")
            || Kind == TEXT("escalator");
    }

    struct FVerticalLanding
    {
        FString NodeId;
        FString SpaceId;
        FString LevelId;
        double Z = 0.0;
        double EdgeDistanceMeters = TNumericLimits<double>::Max();
    };
}

bool UIFCPathSubsystem::FindHierarchicalPathFromWorldPositions(
    const FVector& StartWorldPosition,
    const FVector& GoalWorldPosition,
    TArray<FVector>& OutPoints,
    TArray<FString>& OutSpaceIds,
    TArray<FString>& OutTransitionIds,
    float& OutLengthMeters,
    float& OutWeightedCostMeters) const
{
    OutPoints.Reset();
    OutSpaceIds.Reset();
    OutTransitionIds.Reset();
    OutLengthMeters = 0.0f;
    OutWeightedCostMeters = 0.0f;

    FString StartSpaceId;
    FString GoalSpaceId;
    FVector StartPoint = StartWorldPosition;
    FVector GoalPoint = GoalWorldPosition;

    if (const FIFCPathCell* StartCell = FindCellAtWorldPosition(StartWorldPosition))
    {
        StartSpaceId = StartCell->SpaceId;
        StartPoint.Z = CellCentroid(*StartCell).Z;
    }
    else
    {
        FString StartNodeId;
        if (!FindNearestNode(StartWorldPosition, HierarchySnapDistanceCm, StartNodeId, StartPoint))
        {
            return false;
        }
        const FIFCPathNode* StartNode = Nodes.Find(StartNodeId);
        if (StartNode == nullptr)
        {
            return false;
        }
        StartSpaceId = StartNode->SpaceId;
    }

    if (const FIFCPathCell* GoalCell = FindCellAtWorldPosition(GoalWorldPosition))
    {
        GoalSpaceId = GoalCell->SpaceId;
        GoalPoint.Z = CellCentroid(*GoalCell).Z;
    }
    else
    {
        FString GoalNodeId;
        if (!FindNearestNode(GoalWorldPosition, HierarchySnapDistanceCm, GoalNodeId, GoalPoint))
        {
            return false;
        }
        const FIFCPathNode* GoalNode = Nodes.Find(GoalNodeId);
        if (GoalNode == nullptr)
        {
            return false;
        }
        GoalSpaceId = GoalNode->SpaceId;
    }

    if (StartSpaceId.IsEmpty() || GoalSpaceId.IsEmpty())
    {
        return false;
    }
    if (BlockedSpaces.Contains(GoalSpaceId) && GoalSpaceId != StartSpaceId)
    {
        return false;
    }

    if (StartSpaceId == GoalSpaceId)
    {
        if (!FindLocalPathInSpace(
            StartPoint,
            GoalPoint,
            StartSpaceId,
            HierarchySnapDistanceCm,
            OutPoints))
        {
            return false;
        }
        OutSpaceIds.Add(StartSpaceId);
        OutLengthMeters = static_cast<float>(PolylineLengthCm(OutPoints) / 100.0);
        OutWeightedCostMeters = OutLengthMeters * GetSpaceCostMultiplier(StartSpaceId);
        return true;
    }

    TArray<FIFCPathHierarchyTransfer> Transfers;
    BuildHierarchyTransfers(Transfers);
    if (Transfers.Num() == 0)
    {
        return false;
    }

    TMap<FString, TArray<FIFCPathHierarchyAnchor>> AnchorsBySpace;
    FIFCPathHierarchyAnchor StartAnchor;
    StartAnchor.Id = TEXT("anchor:start");
    StartAnchor.SpaceId = StartSpaceId;
    StartAnchor.Position = StartPoint;
    AnchorsBySpace.FindOrAdd(StartSpaceId).Add(StartAnchor);

    FIFCPathHierarchyAnchor GoalAnchor;
    GoalAnchor.Id = TEXT("anchor:goal");
    GoalAnchor.SpaceId = GoalSpaceId;
    GoalAnchor.Position = GoalPoint;
    AnchorsBySpace.FindOrAdd(GoalSpaceId).Add(GoalAnchor);

    for (const FIFCPathHierarchyTransfer& Transfer : Transfers)
    {
        FIFCPathHierarchyAnchor FromAnchor;
        FromAnchor.Id = FString::Printf(TEXT("anchor:%s:from"), *Transfer.Id);
        FromAnchor.SpaceId = Transfer.FromSpaceId;
        FromAnchor.Position = Transfer.FromPoint;
        AnchorsBySpace.FindOrAdd(Transfer.FromSpaceId).Add(FromAnchor);

        FIFCPathHierarchyAnchor ToAnchor;
        ToAnchor.Id = FString::Printf(TEXT("anchor:%s:to"), *Transfer.Id);
        ToAnchor.SpaceId = Transfer.ToSpaceId;
        ToAnchor.Position = Transfer.ToPoint;
        AnchorsBySpace.FindOrAdd(Transfer.ToSpaceId).Add(ToAnchor);
    }

    TMap<FString, TArray<FIFCPathHierarchyGraphEdge>> Graph;

    for (const TPair<FString, TArray<FIFCPathHierarchyAnchor>>& Pair : AnchorsBySpace)
    {
        const FString& SpaceId = Pair.Key;
        if (BlockedSpaces.Contains(SpaceId) && SpaceId != StartSpaceId)
        {
            continue;
        }

        const TArray<FIFCPathHierarchyAnchor>& SpaceAnchors = Pair.Value;
        const double Multiplier = FMath::Max(1.0, static_cast<double>(GetSpaceCostMultiplier(SpaceId)));
        for (int32 I = 0; I < SpaceAnchors.Num(); ++I)
        {
            for (int32 J = I + 1; J < SpaceAnchors.Num(); ++J)
            {
                const FIFCPathHierarchyAnchor& A = SpaceAnchors[I];
                const FIFCPathHierarchyAnchor& B = SpaceAnchors[J];
                TArray<FVector> LocalPoints;
                if (!FindLocalPathInSpace(
                    A.Position,
                    B.Position,
                    SpaceId,
                    HierarchySnapDistanceCm,
                    LocalPoints))
                {
                    continue;
                }

                const double LocalLengthCm = PolylineLengthCm(LocalPoints);
                const double CostCm = LocalLengthCm * Multiplier;

                FIFCPathHierarchyGraphEdge Forward;
                Forward.TargetId = B.Id;
                Forward.CostCm = CostCm;
                Forward.Kind = TEXT("local");
                Forward.FromSpaceId = SpaceId;
                Forward.ToSpaceId = SpaceId;
                Forward.Points = LocalPoints;
                Graph.FindOrAdd(A.Id).Add(Forward);

                FIFCPathHierarchyGraphEdge Reverse = Forward;
                Reverse.TargetId = A.Id;
                Algo::Reverse(Reverse.Points);
                Graph.FindOrAdd(B.Id).Add(Reverse);
            }
        }
    }

    for (const FIFCPathHierarchyTransfer& Transfer : Transfers)
    {
        if (!Transfer.PortalId.IsEmpty() && BlockedPortals.Contains(Transfer.PortalId))
        {
            continue;
        }

        const FString FromAnchorId = FString::Printf(TEXT("anchor:%s:from"), *Transfer.Id);
        const FString ToAnchorId = FString::Printf(TEXT("anchor:%s:to"), *Transfer.Id);
        const double TransferCostCm = PolylineLengthCm(Transfer.Points);

        if (!BlockedSpaces.Contains(Transfer.ToSpaceId))
        {
            FIFCPathHierarchyGraphEdge Forward;
            Forward.TargetId = ToAnchorId;
            Forward.CostCm = TransferCostCm;
            Forward.Kind = Transfer.Kind;
            Forward.FromSpaceId = Transfer.FromSpaceId;
            Forward.ToSpaceId = Transfer.ToSpaceId;
            Forward.TransitionId = Transfer.Id;
            Forward.PortalId = Transfer.PortalId;
            Forward.Points = Transfer.Points;
            Graph.FindOrAdd(FromAnchorId).Add(Forward);
        }

        if (Transfer.bBidirectional && !BlockedSpaces.Contains(Transfer.FromSpaceId))
        {
            FIFCPathHierarchyGraphEdge Reverse;
            Reverse.TargetId = FromAnchorId;
            Reverse.CostCm = TransferCostCm;
            Reverse.Kind = Transfer.Kind;
            Reverse.FromSpaceId = Transfer.ToSpaceId;
            Reverse.ToSpaceId = Transfer.FromSpaceId;
            Reverse.TransitionId = Transfer.Id;
            Reverse.PortalId = Transfer.PortalId;
            Reverse.Points = Transfer.Points;
            Algo::Reverse(Reverse.Points);
            Graph.FindOrAdd(ToAnchorId).Add(Reverse);
        }
    }

    TSet<FString> AnchorIds;
    AnchorIds.Add(StartAnchor.Id);
    AnchorIds.Add(GoalAnchor.Id);
    for (const TPair<FString, TArray<FIFCPathHierarchyAnchor>>& Pair : AnchorsBySpace)
    {
        for (const FIFCPathHierarchyAnchor& Anchor : Pair.Value)
        {
            AnchorIds.Add(Anchor.Id);
        }
    }

    TMap<FString, double> Dist;
    TMap<FString, FString> PrevNode;
    TMap<FString, FIFCPathHierarchyGraphEdge> PrevEdge;
    TSet<FString> Unvisited = AnchorIds;
    for (const FString& AnchorId : AnchorIds)
    {
        Dist.Add(
            AnchorId,
            AnchorId == StartAnchor.Id ? 0.0 : TNumericLimits<double>::Max());
    }

    while (Unvisited.Num() > 0)
    {
        FString Current;
        double Best = TNumericLimits<double>::Max();
        for (const FString& Candidate : Unvisited)
        {
            const double CandidateDistance = Dist.FindRef(Candidate);
            if (CandidateDistance < Best)
            {
                Best = CandidateDistance;
                Current = Candidate;
            }
        }

        if (Current.IsEmpty() || Best == TNumericLimits<double>::Max())
        {
            break;
        }
        if (Current == GoalAnchor.Id)
        {
            break;
        }
        Unvisited.Remove(Current);

        const TArray<FIFCPathHierarchyGraphEdge>* Neighbours = Graph.Find(Current);
        if (Neighbours == nullptr)
        {
            continue;
        }
        for (const FIFCPathHierarchyGraphEdge& Edge : *Neighbours)
        {
            if (!Unvisited.Contains(Edge.TargetId))
            {
                continue;
            }
            const double Candidate = Best + Edge.CostCm;
            if (Candidate < Dist.FindRef(Edge.TargetId))
            {
                Dist[Edge.TargetId] = Candidate;
                PrevNode.Add(Edge.TargetId, Current);
                PrevEdge.Add(Edge.TargetId, Edge);
            }
        }
    }

    if (!PrevNode.Contains(GoalAnchor.Id))
    {
        return false;
    }

    TArray<FIFCPathHierarchyGraphEdge> RouteEdges;
    FString Cursor = GoalAnchor.Id;
    while (Cursor != StartAnchor.Id)
    {
        const FString* Parent = PrevNode.Find(Cursor);
        const FIFCPathHierarchyGraphEdge* Edge = PrevEdge.Find(Cursor);
        if (Parent == nullptr || Edge == nullptr)
        {
            return false;
        }
        RouteEdges.Add(*Edge);
        Cursor = *Parent;
    }
    Algo::Reverse(RouteEdges);

    OutSpaceIds.Add(StartSpaceId);
    double WeightedCostCm = 0.0;
    for (const FIFCPathHierarchyGraphEdge& Edge : RouteEdges)
    {
        AppendUniquePoints(OutPoints, Edge.Points);
        WeightedCostCm += Edge.CostCm;
        if (!Edge.TransitionId.IsEmpty())
        {
            OutTransitionIds.Add(Edge.TransitionId);
            if (!Edge.ToSpaceId.IsEmpty()
                && (OutSpaceIds.Num() == 0 || OutSpaceIds.Last() != Edge.ToSpaceId))
            {
                OutSpaceIds.Add(Edge.ToSpaceId);
            }
        }
    }

    if (OutPoints.Num() < 2)
    {
        return false;
    }

    OutLengthMeters = static_cast<float>(PolylineLengthCm(OutPoints) / 100.0);
    OutWeightedCostMeters = static_cast<float>(WeightedCostCm / 100.0);
    return true;
}

void UIFCPathSubsystem::BuildHierarchyTransfers(TArray<FIFCPathHierarchyTransfer>& OutTransfers) const
{
    OutTransfers.Reset();

    // Horizontal semantic transfers are recovered from qualified portal edges.
    // This keeps the runtime backward-compatible with older INAV files that do
    // not yet materialize a separate transition array in the Unreal loader.
    TMap<FString, FVector> PortalPositions;
    TMap<FString, TSet<FString>> PortalSpaces;
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        const FIFCPathNode& Node = Pair.Value;
        if (!Node.PortalId.IsEmpty() && Node.Kind == TEXT("portal"))
        {
            PortalPositions.Add(Node.PortalId, Node.Position);
        }
    }

    for (const FIFCPathEdge& Edge : Edges)
    {
        if (Edge.PortalId.IsEmpty())
        {
            continue;
        }
        const FIFCPathNode* A = Nodes.Find(Edge.A);
        const FIFCPathNode* B = Nodes.Find(Edge.B);
        if (A != nullptr && !A->SpaceId.IsEmpty())
        {
            PortalSpaces.FindOrAdd(Edge.PortalId).Add(A->SpaceId);
        }
        if (B != nullptr && !B->SpaceId.IsEmpty())
        {
            PortalSpaces.FindOrAdd(Edge.PortalId).Add(B->SpaceId);
        }
    }

    for (const TPair<FString, TSet<FString>>& Pair : PortalSpaces)
    {
        const FVector* PortalPosition = PortalPositions.Find(Pair.Key);
        if (PortalPosition == nullptr || Pair.Value.Num() < 2)
        {
            continue;
        }
        TArray<FString> Spaces = Pair.Value.Array();
        Spaces.Sort();
        const FString FromSpaceId = Spaces[0];
        const FString ToSpaceId = Spaces[1];

        FVector FromPoint;
        FVector ToPoint;
        if (!SnapPointToSpaceNavMesh(
            *PortalPosition,
            FromSpaceId,
            HierarchySnapDistanceCm,
            FromPoint)
            || !SnapPointToSpaceNavMesh(
                *PortalPosition,
                ToSpaceId,
                HierarchySnapDistanceCm,
                ToPoint))
        {
            continue;
        }

        FIFCPathHierarchyTransfer Transfer;
        Transfer.Id = FString::Printf(TEXT("transition:%s"), *Pair.Key);
        Transfer.Kind = TEXT("door");
        Transfer.PortalId = Pair.Key;
        Transfer.FromSpaceId = FromSpaceId;
        Transfer.ToSpaceId = ToSpaceId;
        Transfer.FromPoint = FromPoint;
        Transfer.ToPoint = ToPoint;
        AppendUniquePoints(Transfer.Points, {FromPoint, *PortalPosition, ToPoint});
        OutTransfers.Add(MoveTemp(Transfer));
    }

    // Build metric adjacency once for geometry-assisted vertical semantics.
    TMap<FString, TArray<TPair<FString, double>>> MetricAdjacency;
    TSet<FString> VerticalIds;
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        if (IsVerticalKind(Pair.Value.Kind))
        {
            VerticalIds.Add(Pair.Key);
        }
    }
    for (const FIFCPathEdge& Edge : Edges)
    {
        if (!Nodes.Contains(Edge.A) || !Nodes.Contains(Edge.B))
        {
            continue;
        }
        MetricAdjacency.FindOrAdd(Edge.A).Add(TPair<FString, double>(Edge.B, Edge.DistanceMeters));
        MetricAdjacency.FindOrAdd(Edge.B).Add(TPair<FString, double>(Edge.A, Edge.DistanceMeters));
    }

    TSet<FString> Remaining = VerticalIds;
    while (Remaining.Num() > 0)
    {
        const FString Start = *Remaining.CreateConstIterator();
        Remaining.Remove(Start);
        TSet<FString> Component;
        Component.Add(Start);
        TArray<FString> Queue;
        Queue.Add(Start);
        int32 Head = 0;
        while (Head < Queue.Num())
        {
            const FString Current = Queue[Head++];
            const TArray<TPair<FString, double>>* Neighbours = MetricAdjacency.Find(Current);
            if (Neighbours == nullptr)
            {
                continue;
            }
            for (const TPair<FString, double>& Neighbour : *Neighbours)
            {
                if (Remaining.Contains(Neighbour.Key) && VerticalIds.Contains(Neighbour.Key))
                {
                    Remaining.Remove(Neighbour.Key);
                    Component.Add(Neighbour.Key);
                    Queue.Add(Neighbour.Key);
                }
            }
        }

        FString Kind;
        bool bMixedKind = false;
        for (const FString& NodeId : Component)
        {
            const FIFCPathNode* Node = Nodes.Find(NodeId);
            if (Node == nullptr)
            {
                continue;
            }
            if (Kind.IsEmpty())
            {
                Kind = Node->Kind;
            }
            else if (Kind != Node->Kind)
            {
                bMixedKind = true;
            }
        }
        if (bMixedKind)
        {
            Kind = TEXT("vertical");
        }

        TMap<FString, FVerticalLanding> LandingByLevel;
        for (const FString& VerticalId : Component)
        {
            const TArray<TPair<FString, double>>* Neighbours = MetricAdjacency.Find(VerticalId);
            if (Neighbours == nullptr)
            {
                continue;
            }
            for (const TPair<FString, double>& Neighbour : *Neighbours)
            {
                if (Component.Contains(Neighbour.Key))
                {
                    continue;
                }
                const FIFCPathNode* LandingNode = Nodes.Find(Neighbour.Key);
                if (LandingNode == nullptr || LandingNode->SpaceId.IsEmpty())
                {
                    continue;
                }
                const FString LevelKey = LandingNode->LevelId.IsEmpty()
                    ? LandingNode->SpaceId
                    : LandingNode->LevelId;
                FVerticalLanding* Existing = LandingByLevel.Find(LevelKey);
                if (Existing == nullptr || Neighbour.Value < Existing->EdgeDistanceMeters)
                {
                    FVerticalLanding Landing;
                    Landing.NodeId = Neighbour.Key;
                    Landing.SpaceId = LandingNode->SpaceId;
                    Landing.LevelId = LandingNode->LevelId;
                    Landing.Z = LandingNode->Position.Z;
                    Landing.EdgeDistanceMeters = Neighbour.Value;
                    LandingByLevel.Add(LevelKey, Landing);
                }
            }
        }

        if (LandingByLevel.Num() < 2)
        {
            continue;
        }

        TArray<FVerticalLanding> Landings;
        LandingByLevel.GenerateValueArray(Landings);
        Landings.Sort([](const FVerticalLanding& A, const FVerticalLanding& B)
        {
            if (!FMath::IsNearlyEqual(A.Z, B.Z))
            {
                return A.Z < B.Z;
            }
            return A.SpaceId < B.SpaceId;
        });

        TArray<FString> ComponentIds = Component.Array();
        ComponentIds.Sort();
        const FString Representative = ComponentIds.Num() > 0 ? ComponentIds[0] : TEXT("vertical");

        for (int32 Index = 1; Index < Landings.Num(); ++Index)
        {
            const FVerticalLanding& FromLanding = Landings[Index - 1];
            const FVerticalLanding& ToLanding = Landings[Index];
            if (FromLanding.SpaceId == ToLanding.SpaceId)
            {
                continue;
            }

            TArray<FVector> VerticalPoints;
            if (!FindVerticalTransferPath(
                Component,
                FromLanding.NodeId,
                ToLanding.NodeId,
                VerticalPoints))
            {
                continue;
            }

            FVector FromPoint;
            FVector ToPoint;
            if (!SnapPointToSpaceNavMesh(
                VerticalPoints[0],
                FromLanding.SpaceId,
                HierarchySnapDistanceCm,
                FromPoint)
                || !SnapPointToSpaceNavMesh(
                    VerticalPoints.Last(),
                    ToLanding.SpaceId,
                    HierarchySnapDistanceCm,
                    ToPoint))
            {
                continue;
            }

            FIFCPathHierarchyTransfer Transfer;
            Transfer.Id = FString::Printf(
                TEXT("transition:vertical:%s:%s:%s:%s"),
                *Kind,
                *FromLanding.SpaceId,
                *ToLanding.SpaceId,
                *Representative);
            Transfer.Kind = Kind;
            Transfer.FromSpaceId = FromLanding.SpaceId;
            Transfer.ToSpaceId = ToLanding.SpaceId;
            Transfer.FromPoint = FromPoint;
            Transfer.ToPoint = ToPoint;
            Transfer.Points.Add(FromPoint);
            AppendUniquePoints(Transfer.Points, VerticalPoints);
            TArray<FVector> ToArray;
            ToArray.Add(ToPoint);
            AppendUniquePoints(Transfer.Points, ToArray);
            OutTransfers.Add(MoveTemp(Transfer));
        }
    }
}

bool UIFCPathSubsystem::FindVerticalTransferPath(
    const TSet<FString>& Component,
    const FString& FromLandingId,
    const FString& ToLandingId,
    TArray<FVector>& OutPoints) const
{
    OutPoints.Reset();
    if (!Nodes.Contains(FromLandingId) || !Nodes.Contains(ToLandingId))
    {
        return false;
    }

    TSet<FString> Allowed = Component;
    Allowed.Add(FromLandingId);
    Allowed.Add(ToLandingId);

    TMap<FString, TArray<TPair<FString, double>>> Restricted;
    for (const FIFCPathEdge& Edge : Edges)
    {
        if (!Allowed.Contains(Edge.A) || !Allowed.Contains(Edge.B))
        {
            continue;
        }
        if (!Component.Contains(Edge.A) && !Component.Contains(Edge.B))
        {
            continue;
        }
        Restricted.FindOrAdd(Edge.A).Add(TPair<FString, double>(Edge.B, Edge.DistanceMeters));
        Restricted.FindOrAdd(Edge.B).Add(TPair<FString, double>(Edge.A, Edge.DistanceMeters));
    }

    TMap<FString, double> Dist;
    TMap<FString, FString> Prev;
    TSet<FString> Unvisited = Allowed;
    for (const FString& NodeId : Allowed)
    {
        Dist.Add(
            NodeId,
            NodeId == FromLandingId ? 0.0 : TNumericLimits<double>::Max());
    }

    while (Unvisited.Num() > 0)
    {
        FString Current;
        double Best = TNumericLimits<double>::Max();
        for (const FString& Candidate : Unvisited)
        {
            const double CandidateDistance = Dist.FindRef(Candidate);
            if (CandidateDistance < Best)
            {
                Best = CandidateDistance;
                Current = Candidate;
            }
        }
        if (Current.IsEmpty() || Best == TNumericLimits<double>::Max())
        {
            break;
        }
        if (Current == ToLandingId)
        {
            break;
        }
        Unvisited.Remove(Current);

        const TArray<TPair<FString, double>>* Neighbours = Restricted.Find(Current);
        if (Neighbours == nullptr)
        {
            continue;
        }
        for (const TPair<FString, double>& Neighbour : *Neighbours)
        {
            if (!Unvisited.Contains(Neighbour.Key))
            {
                continue;
            }
            const double Candidate = Best + Neighbour.Value;
            if (Candidate < Dist.FindRef(Neighbour.Key))
            {
                Dist[Neighbour.Key] = Candidate;
                Prev.Add(Neighbour.Key, Current);
            }
        }
    }

    if (!Prev.Contains(ToLandingId))
    {
        return false;
    }

    TArray<FString> PathIds;
    FString Cursor = ToLandingId;
    PathIds.Add(Cursor);
    while (Cursor != FromLandingId)
    {
        const FString* Parent = Prev.Find(Cursor);
        if (Parent == nullptr)
        {
            return false;
        }
        Cursor = *Parent;
        PathIds.Add(Cursor);
    }
    Algo::Reverse(PathIds);

    for (const FString& NodeId : PathIds)
    {
        if (const FIFCPathNode* Node = Nodes.Find(NodeId))
        {
            OutPoints.Add(Node->Position);
        }
    }
    return OutPoints.Num() >= 2;
}

const FIFCPathCell* UIFCPathSubsystem::FindCellAtWorldPositionInSpace(
    const FVector& WorldPosition,
    const FString& SpaceId) const
{
    const FIFCPathCell* BestCell = nullptr;
    double BestZDistance = TNumericLimits<double>::Max();
    for (const TPair<FString, FIFCPathCell>& Pair : Cells)
    {
        const FIFCPathCell& Cell = Pair.Value;
        if (Cell.SpaceId != SpaceId || Cell.Vertices.Num() != 3 || !PointInCell2D(WorldPosition, Cell))
        {
            continue;
        }
        const double ZDistance = FMath::Abs(WorldPosition.Z - CellCentroid(Cell).Z);
        if (ZDistance <= HierarchyCellPlaneToleranceCm && ZDistance < BestZDistance)
        {
            BestZDistance = ZDistance;
            BestCell = &Cell;
        }
    }
    return BestCell;
}

bool UIFCPathSubsystem::SnapPointToSpaceNavMesh(
    const FVector& WorldPosition,
    const FString& SpaceId,
    double MaxDistanceCm,
    FVector& OutPoint) const
{
    if (const FIFCPathCell* Cell = FindCellAtWorldPositionInSpace(WorldPosition, SpaceId))
    {
        OutPoint = WorldPosition;
        OutPoint.Z = CellCentroid(*Cell).Z;
        return FVector::Dist(OutPoint, WorldPosition) <= MaxDistanceCm;
    }

    bool bFound = false;
    double BestDistance = TNumericLimits<double>::Max();
    FVector BestPoint = FVector::ZeroVector;
    for (const TPair<FString, FIFCPathCell>& Pair : Cells)
    {
        const FIFCPathCell& Cell = Pair.Value;
        if (Cell.SpaceId != SpaceId || Cell.Vertices.Num() != 3)
        {
            continue;
        }
        for (int32 Index = 0; Index < 3; ++Index)
        {
            const FVector Candidate = ClosestPointOnSegment2D(
                WorldPosition,
                Cell.Vertices[Index],
                Cell.Vertices[(Index + 1) % 3]);
            const double CandidateDistance = FVector::Dist(WorldPosition, Candidate);
            if (CandidateDistance < BestDistance)
            {
                BestDistance = CandidateDistance;
                BestPoint = Candidate;
                bFound = true;
            }
        }
    }

    if (bFound && BestDistance <= MaxDistanceCm)
    {
        OutPoint = BestPoint;
        return true;
    }

    // Sampled-floor fallback.
    const FIFCPathNode* BestNode = nullptr;
    BestDistance = TNumericLimits<double>::Max();
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        if (Pair.Value.SpaceId != SpaceId)
        {
            continue;
        }
        const double CandidateDistance = FVector::Dist(WorldPosition, Pair.Value.Position);
        if (CandidateDistance < BestDistance)
        {
            BestDistance = CandidateDistance;
            BestNode = &Pair.Value;
        }
    }
    if (BestNode != nullptr && BestDistance <= MaxDistanceCm)
    {
        OutPoint = BestNode->Position;
        return true;
    }
    return false;
}

bool UIFCPathSubsystem::FindNavMeshPathInSpace(
    const FVector& StartWorldPosition,
    const FVector& GoalWorldPosition,
    const FString& SpaceId,
    TArray<FVector>& OutPoints) const
{
    OutPoints.Reset();
    const FIFCPathCell* StartCell = FindCellAtWorldPositionInSpace(StartWorldPosition, SpaceId);
    const FIFCPathCell* GoalCell = FindCellAtWorldPositionInSpace(GoalWorldPosition, SpaceId);
    if (StartCell == nullptr || GoalCell == nullptr)
    {
        return false;
    }

    if (StartCell->Id == GoalCell->Id)
    {
        OutPoints.Add(StartWorldPosition);
        OutPoints.Add(GoalWorldPosition);
        return true;
    }

    TArray<FString> Corridor;
    if (!FindCellCorridor(StartCell->Id, GoalCell->Id, SpaceId, Corridor))
    {
        return false;
    }

    TArray<TPair<FVector, FVector>> Portals;
    Portals.Add(TPair<FVector, FVector>(StartWorldPosition, StartWorldPosition));
    for (int32 Index = 1; Index < Corridor.Num(); ++Index)
    {
        const FIFCPathCell* Current = Cells.Find(Corridor[Index - 1]);
        const FIFCPathCell* Next = Cells.Find(Corridor[Index]);
        if (Current == nullptr || Next == nullptr)
        {
            return false;
        }
        FVector A;
        FVector B;
        if (!GetSharedCellEdge(*Current, *Next, A, B))
        {
            return false;
        }
        Portals.Add(OrientPortal(*Current, *Next, A, B));
    }
    Portals.Add(TPair<FVector, FVector>(GoalWorldPosition, GoalWorldPosition));
    StringPull(Portals, OutPoints);
    return OutPoints.Num() >= 2;
}

bool UIFCPathSubsystem::FindLocalPathInSpace(
    const FVector& StartWorldPosition,
    const FVector& GoalWorldPosition,
    const FString& SpaceId,
    double MaxSnapDistanceCm,
    TArray<FVector>& OutPoints) const
{
    if (FindNavMeshPathInSpace(StartWorldPosition, GoalWorldPosition, SpaceId, OutPoints))
    {
        return true;
    }

    const FIFCPathNode* StartNode = nullptr;
    const FIFCPathNode* GoalNode = nullptr;
    double StartDistance = TNumericLimits<double>::Max();
    double GoalDistance = TNumericLimits<double>::Max();
    TSet<FString> LocalNodeIds;
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        if (Pair.Value.SpaceId != SpaceId)
        {
            continue;
        }
        LocalNodeIds.Add(Pair.Key);
        const double ToStart = FVector::Dist(StartWorldPosition, Pair.Value.Position);
        if (ToStart < StartDistance)
        {
            StartDistance = ToStart;
            StartNode = &Pair.Value;
        }
        const double ToGoal = FVector::Dist(GoalWorldPosition, Pair.Value.Position);
        if (ToGoal < GoalDistance)
        {
            GoalDistance = ToGoal;
            GoalNode = &Pair.Value;
        }
    }

    if (StartNode == nullptr || GoalNode == nullptr
        || StartDistance > MaxSnapDistanceCm || GoalDistance > MaxSnapDistanceCm)
    {
        return false;
    }

    TMap<FString, TArray<TPair<FString, double>>> LocalAdjacency;
    for (const FIFCPathEdge& Edge : Edges)
    {
        if (LocalNodeIds.Contains(Edge.A) && LocalNodeIds.Contains(Edge.B))
        {
            LocalAdjacency.FindOrAdd(Edge.A).Add(TPair<FString, double>(Edge.B, Edge.DistanceMeters));
            LocalAdjacency.FindOrAdd(Edge.B).Add(TPair<FString, double>(Edge.A, Edge.DistanceMeters));
        }
    }

    TMap<FString, double> Dist;
    TMap<FString, FString> Prev;
    TSet<FString> Unvisited = LocalNodeIds;
    for (const FString& NodeId : LocalNodeIds)
    {
        Dist.Add(
            NodeId,
            NodeId == StartNode->Id ? 0.0 : TNumericLimits<double>::Max());
    }

    while (Unvisited.Num() > 0)
    {
        FString Current;
        double Best = TNumericLimits<double>::Max();
        for (const FString& Candidate : Unvisited)
        {
            const double CandidateDistance = Dist.FindRef(Candidate);
            if (CandidateDistance < Best)
            {
                Best = CandidateDistance;
                Current = Candidate;
            }
        }
        if (Current.IsEmpty() || Best == TNumericLimits<double>::Max())
        {
            break;
        }
        if (Current == GoalNode->Id)
        {
            break;
        }
        Unvisited.Remove(Current);

        const TArray<TPair<FString, double>>* Neighbours = LocalAdjacency.Find(Current);
        if (Neighbours == nullptr)
        {
            continue;
        }
        for (const TPair<FString, double>& Neighbour : *Neighbours)
        {
            if (!Unvisited.Contains(Neighbour.Key))
            {
                continue;
            }
            const double Candidate = Best + Neighbour.Value;
            if (Candidate < Dist.FindRef(Neighbour.Key))
            {
                Dist[Neighbour.Key] = Candidate;
                Prev.Add(Neighbour.Key, Current);
            }
        }
    }

    if (StartNode->Id != GoalNode->Id && !Prev.Contains(GoalNode->Id))
    {
        return false;
    }

    TArray<FString> PathIds;
    FString Cursor = GoalNode->Id;
    PathIds.Add(Cursor);
    while (Cursor != StartNode->Id)
    {
        const FString* Parent = Prev.Find(Cursor);
        if (Parent == nullptr)
        {
            return false;
        }
        Cursor = *Parent;
        PathIds.Add(Cursor);
    }
    Algo::Reverse(PathIds);

    OutPoints.Reset();
    OutPoints.Add(StartWorldPosition);
    for (const FString& NodeId : PathIds)
    {
        if (const FIFCPathNode* Node = Nodes.Find(NodeId))
        {
            TArray<FVector> One;
            One.Add(Node->Position);
            AppendUniquePoints(OutPoints, One);
        }
    }
    TArray<FVector> GoalArray;
    GoalArray.Add(GoalWorldPosition);
    AppendUniquePoints(OutPoints, GoalArray);
    return OutPoints.Num() >= 2;
}

FVector UIFCPathSubsystem::ClosestPointOnSegment2D(
    const FVector& Point,
    const FVector& A,
    const FVector& B)
{
    const FVector2D AB(B.X - A.X, B.Y - A.Y);
    const double Denominator = AB.SizeSquared();
    if (Denominator <= KINDA_SMALL_NUMBER)
    {
        return A;
    }
    const FVector2D AP(Point.X - A.X, Point.Y - A.Y);
    const double T = FMath::Clamp(FVector2D::DotProduct(AP, AB) / Denominator, 0.0, 1.0);
    return FVector(
        A.X + (B.X - A.X) * T,
        A.Y + (B.Y - A.Y) * T,
        A.Z + (B.Z - A.Z) * T);
}

double UIFCPathSubsystem::PolylineLengthCm(const TArray<FVector>& Points)
{
    double Length = 0.0;
    for (int32 Index = 1; Index < Points.Num(); ++Index)
    {
        Length += FVector::Dist(Points[Index - 1], Points[Index]);
    }
    return Length;
}

void UIFCPathSubsystem::AppendUniquePoints(
    TArray<FVector>& Target,
    const TArray<FVector>& Source)
{
    for (const FVector& Point : Source)
    {
        if (Target.Num() == 0 || FVector::Dist(Target.Last(), Point) > HierarchyPointToleranceCm)
        {
            Target.Add(Point);
        }
    }
}
