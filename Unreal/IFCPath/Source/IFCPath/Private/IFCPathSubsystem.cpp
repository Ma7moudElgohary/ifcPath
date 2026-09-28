#include "IFCPathSubsystem.h"

#include "Algo/Reverse.h"
#include "Dom/JsonObject.h"
#include "DrawDebugHelpers.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    constexpr double FunnelEpsilon = 1e-6;
    constexpr double CellPlaneToleranceCm = 200.0;
}

bool UIFCPathSubsystem::LoadInav(const FString& FilePath, FString& Error)
{
    Nodes.Reset();
    Edges.Reset();
    Adjacency.Reset();
    Cells.Reset();
    ClearDynamicState();

    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *FilePath))
    {
        Error = FString::Printf(TEXT("Could not read INAV file: %s"), *FilePath);
        return false;
    }

    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    {
        Error = TEXT("Invalid INAV JSON");
        return false;
    }

    const TArray<TSharedPtr<FJsonValue>>* JsonCells = nullptr;
    if (Root->TryGetArrayField(TEXT("cells"), JsonCells) && JsonCells != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *JsonCells)
        {
            const TSharedPtr<FJsonObject> Obj = Value->AsObject();
            if (!Obj.IsValid())
            {
                continue;
            }

            FIFCPathCell Cell;
            if (!Obj->TryGetStringField(TEXT("id"), Cell.Id) || Cell.Id.IsEmpty())
            {
                continue;
            }
            Obj->TryGetStringField(TEXT("space_id"), Cell.SpaceId);
            Obj->TryGetStringField(TEXT("level_id"), Cell.LevelId);

            const TArray<TSharedPtr<FJsonValue>>* JsonVertices = nullptr;
            if (!Obj->TryGetArrayField(TEXT("vertices_m"), JsonVertices) || JsonVertices == nullptr)
            {
                continue;
            }
            for (const TSharedPtr<FJsonValue>& VertexValue : *JsonVertices)
            {
                const TArray<TSharedPtr<FJsonValue>>& Coordinates = VertexValue->AsArray();
                if (Coordinates.Num() < 3)
                {
                    continue;
                }
                Cell.Vertices.Add(ToUnrealPosition(
                    Coordinates[0]->AsNumber(),
                    Coordinates[1]->AsNumber(),
                    Coordinates[2]->AsNumber()));
            }
            if (Cell.Vertices.Num() != 3)
            {
                continue;
            }

            const TArray<TSharedPtr<FJsonValue>>* JsonNeighbours = nullptr;
            if (Obj->TryGetArrayField(TEXT("neighbor_ids"), JsonNeighbours) && JsonNeighbours != nullptr)
            {
                for (const TSharedPtr<FJsonValue>& Neighbour : *JsonNeighbours)
                {
                    Cell.NeighborIds.Add(Neighbour->AsString());
                }
            }

            Cells.Add(Cell.Id, MoveTemp(Cell));
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* JsonNodes = nullptr;
    if (!Root->TryGetArrayField(TEXT("nodes"), JsonNodes) || JsonNodes == nullptr)
    {
        Error = TEXT("INAV has no nodes array");
        return false;
    }

    for (const TSharedPtr<FJsonValue>& Value : *JsonNodes)
    {
        const TSharedPtr<FJsonObject> Obj = Value->AsObject();
        if (!Obj.IsValid())
        {
            continue;
        }

        const TArray<TSharedPtr<FJsonValue>>* Position = nullptr;
        if (!Obj->TryGetArrayField(TEXT("position_m"), Position) || Position == nullptr || Position->Num() < 3)
        {
            continue;
        }

        FIFCPathNode Node;
        if (!Obj->TryGetStringField(TEXT("id"), Node.Id) || Node.Id.IsEmpty())
        {
            continue;
        }
        Node.Position = ToUnrealPosition(
            (*Position)[0]->AsNumber(),
            (*Position)[1]->AsNumber(),
            (*Position)[2]->AsNumber());
        Obj->TryGetStringField(TEXT("kind"), Node.Kind);
        Obj->TryGetStringField(TEXT("level_id"), Node.LevelId);
        Obj->TryGetStringField(TEXT("space_id"), Node.SpaceId);
        Obj->TryGetStringField(TEXT("portal_id"), Node.PortalId);
        Obj->TryGetStringField(TEXT("cell_id"), Node.CellId);

        Nodes.Add(Node.Id, MoveTemp(Node));
    }

    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        Adjacency.Add(Pair.Key, TArray<FIFCPathAdjacencyEntry>());
    }

    const TArray<TSharedPtr<FJsonValue>>* JsonEdges = nullptr;
    if (Root->TryGetArrayField(TEXT("edges"), JsonEdges) && JsonEdges != nullptr)
    {
        for (const TSharedPtr<FJsonValue>& Value : *JsonEdges)
        {
            const TSharedPtr<FJsonObject> Obj = Value->AsObject();
            if (!Obj.IsValid())
            {
                continue;
            }

            FIFCPathEdge Edge;
            if (!Obj->TryGetStringField(TEXT("a"), Edge.A)
                || !Obj->TryGetStringField(TEXT("b"), Edge.B)
                || !Nodes.Contains(Edge.A)
                || !Nodes.Contains(Edge.B))
            {
                continue;
            }
            Edge.DistanceMeters = Obj->GetNumberField(TEXT("distance_m"));
            Obj->TryGetStringField(TEXT("portal_id"), Edge.PortalId);
            Edges.Add(Edge);

            FIFCPathAdjacencyEntry AB;
            AB.NodeId = Edge.B;
            AB.DistanceMeters = Edge.DistanceMeters;
            AB.PortalId = Edge.PortalId;
            Adjacency.FindChecked(Edge.A).Add(MoveTemp(AB));

            FIFCPathAdjacencyEntry BA;
            BA.NodeId = Edge.A;
            BA.DistanceMeters = Edge.DistanceMeters;
            BA.PortalId = Edge.PortalId;
            Adjacency.FindChecked(Edge.B).Add(MoveTemp(BA));
        }
    }

    Error.Reset();
    return Nodes.Num() > 0;
}

bool UIFCPathSubsystem::FindPath(
    const FString& StartNodeId,
    const FString& GoalNodeId,
    TArray<FVector>& OutPoints) const
{
    OutPoints.Reset();
    const FIFCPathNode* StartNode = Nodes.Find(StartNodeId);
    const FIFCPathNode* GoalNode = Nodes.Find(GoalNodeId);
    if (StartNode == nullptr || GoalNode == nullptr)
    {
        return false;
    }

    const FString StartSpaceId = StartNode->SpaceId;
    if (!GoalNode->SpaceId.IsEmpty()
        && BlockedSpaces.Contains(GoalNode->SpaceId)
        && GoalNode->SpaceId != StartSpaceId)
    {
        return false;
    }

    TMap<FString, double> Dist;
    TMap<FString, FString> Prev;
    TSet<FString> Unvisited;
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        Dist.Add(Pair.Key, Pair.Key == StartNodeId ? 0.0 : TNumericLimits<double>::Max());
        Unvisited.Add(Pair.Key);
    }

    while (Unvisited.Num() > 0)
    {
        FString Current;
        double Best = TNumericLimits<double>::Max();
        for (const FString& Candidate : Unvisited)
        {
            const double CandidateDist = Dist.FindRef(Candidate);
            if (CandidateDist < Best)
            {
                Best = CandidateDist;
                Current = Candidate;
            }
        }

        if (Current.IsEmpty() || Best == TNumericLimits<double>::Max())
        {
            break;
        }
        if (Current == GoalNodeId)
        {
            break;
        }
        Unvisited.Remove(Current);

        const FIFCPathNode* CurrentNode = Nodes.Find(Current);
        const TArray<FIFCPathAdjacencyEntry>* Neighbours = Adjacency.Find(Current);
        if (CurrentNode == nullptr || Neighbours == nullptr)
        {
            continue;
        }

        for (const FIFCPathAdjacencyEntry& Neighbour : *Neighbours)
        {
            if (!Neighbour.PortalId.IsEmpty() && BlockedPortals.Contains(Neighbour.PortalId))
            {
                continue;
            }
            if (!Unvisited.Contains(Neighbour.NodeId))
            {
                continue;
            }

            const FIFCPathNode* NextNode = Nodes.Find(Neighbour.NodeId);
            if (NextNode == nullptr)
            {
                continue;
            }

            if (!NextNode->SpaceId.IsEmpty()
                && BlockedSpaces.Contains(NextNode->SpaceId)
                && NextNode->SpaceId != StartSpaceId)
            {
                continue;
            }

            const double CandidateDist = Best
                + Neighbour.DistanceMeters * GetTraversalMultiplier(*CurrentNode, *NextNode);
            if (CandidateDist < Dist.FindRef(Neighbour.NodeId))
            {
                Dist[Neighbour.NodeId] = CandidateDist;
                Prev.Add(Neighbour.NodeId, Current);
            }
        }
    }

    if (StartNodeId != GoalNodeId && !Prev.Contains(GoalNodeId))
    {
        return false;
    }

    TArray<FString> Ids;
    FString Cursor = GoalNodeId;
    Ids.Add(Cursor);
    while (Cursor != StartNodeId)
    {
        const FString* Parent = Prev.Find(Cursor);
        if (Parent == nullptr)
        {
            return false;
        }
        Cursor = *Parent;
        Ids.Add(Cursor);
    }

    Algo::Reverse(Ids);
    for (const FString& Id : Ids)
    {
        if (const FIFCPathNode* Node = Nodes.Find(Id))
        {
            OutPoints.Add(Node->Position);
        }
    }
    return OutPoints.Num() > 0;
}

bool UIFCPathSubsystem::FindNearestNode(
    const FVector& WorldPosition,
    float MaxDistanceCm,
    FString& OutNodeId,
    FVector& OutNodePosition) const
{
    OutNodeId.Reset();
    OutNodePosition = FVector::ZeroVector;

    const double MaxDistanceSquared = MaxDistanceCm > 0.0f
        ? FMath::Square(static_cast<double>(MaxDistanceCm))
        : TNumericLimits<double>::Max();

    double BestDistanceSquared = MaxDistanceSquared;
    const FIFCPathNode* BestNode = nullptr;
    for (const TPair<FString, FIFCPathNode>& Pair : Nodes)
    {
        const double DistanceSquared = FVector::DistSquared(WorldPosition, Pair.Value.Position);
        if (DistanceSquared <= BestDistanceSquared)
        {
            BestDistanceSquared = DistanceSquared;
            BestNode = &Pair.Value;
        }
    }

    if (BestNode == nullptr)
    {
        return false;
    }

    OutNodeId = BestNode->Id;
    OutNodePosition = BestNode->Position;
    return true;
}

bool UIFCPathSubsystem::FindPathFromWorldPositions(
    const FVector& StartWorldPosition,
    const FVector& GoalWorldPosition,
    float MaxSnapDistanceCm,
    TArray<FVector>& OutPoints,
    FString& OutStartNodeId,
    FString& OutGoalNodeId) const
{
    FVector SnappedStart;
    FVector SnappedGoal;
    if (!FindNearestNode(StartWorldPosition, MaxSnapDistanceCm, OutStartNodeId, SnappedStart))
    {
        return false;
    }
    if (!FindNearestNode(GoalWorldPosition, MaxSnapDistanceCm, OutGoalNodeId, SnappedGoal))
    {
        return false;
    }
    return FindPath(OutStartNodeId, OutGoalNodeId, OutPoints);
}

bool UIFCPathSubsystem::FindNavMeshPathFromWorldPositions(
    const FVector& StartWorldPosition,
    const FVector& GoalWorldPosition,
    TArray<FVector>& OutPoints,
    FString& OutSpaceId) const
{
    OutPoints.Reset();
    OutSpaceId.Reset();

    const FIFCPathCell* StartCell = FindCellAtWorldPosition(StartWorldPosition);
    const FIFCPathCell* GoalCell = FindCellAtWorldPosition(GoalWorldPosition);
    if (StartCell == nullptr || GoalCell == nullptr)
    {
        return false;
    }
    if (StartCell->SpaceId.IsEmpty() || StartCell->SpaceId != GoalCell->SpaceId)
    {
        return false;
    }

    OutSpaceId = StartCell->SpaceId;
    if (BlockedSpaces.Contains(OutSpaceId))
    {
        // The local query has no concept of an external destination to escape
        // toward. Building-wide escape from a blocked start space remains owned
        // by the semantic/metric graph pathfinder.
        return false;
    }

    if (StartCell->Id == GoalCell->Id)
    {
        OutPoints.Add(StartWorldPosition);
        OutPoints.Add(GoalWorldPosition);
        return true;
    }

    TArray<FString> Corridor;
    if (!FindCellCorridor(StartCell->Id, GoalCell->Id, OutSpaceId, Corridor))
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

void UIFCPathSubsystem::SetPortalBlocked(const FString& PortalId, bool bBlocked)
{
    if (bBlocked)
    {
        BlockedPortals.Add(PortalId);
    }
    else
    {
        BlockedPortals.Remove(PortalId);
    }
}

void UIFCPathSubsystem::SetSpaceBlocked(const FString& SpaceId, bool bBlocked)
{
    if (SpaceId.IsEmpty())
    {
        return;
    }
    if (bBlocked)
    {
        BlockedSpaces.Add(SpaceId);
    }
    else
    {
        BlockedSpaces.Remove(SpaceId);
    }
}

void UIFCPathSubsystem::SetSpaceCostMultiplier(const FString& SpaceId, float CostMultiplier)
{
    if (SpaceId.IsEmpty())
    {
        return;
    }

    const double Value = FMath::Max(1.0, static_cast<double>(CostMultiplier));
    if (FMath::IsNearlyEqual(Value, 1.0))
    {
        SpaceCostMultipliers.Remove(SpaceId);
    }
    else
    {
        SpaceCostMultipliers.Add(SpaceId, Value);
    }
}

void UIFCPathSubsystem::ClearDynamicState()
{
    BlockedPortals.Reset();
    BlockedSpaces.Reset();
    SpaceCostMultipliers.Reset();
}

bool UIFCPathSubsystem::IsSpaceBlocked(const FString& SpaceId) const
{
    return BlockedSpaces.Contains(SpaceId);
}

float UIFCPathSubsystem::GetSpaceCostMultiplier(const FString& SpaceId) const
{
    if (const double* Value = SpaceCostMultipliers.Find(SpaceId))
    {
        return static_cast<float>(*Value);
    }
    return 1.0f;
}

void UIFCPathSubsystem::DrawDebugPath(
    const TArray<FVector>& Points,
    FLinearColor Color,
    float Thickness,
    float Duration) const
{
    UWorld* World = GetWorld();
    if (World == nullptr || Points.Num() < 2)
    {
        return;
    }

    const FColor DrawColor = Color.ToFColor(true);
    for (int32 Index = 1; Index < Points.Num(); ++Index)
    {
        DrawDebugLine(World, Points[Index - 1], Points[Index], DrawColor, false, Duration, 0, Thickness);
    }
}

void UIFCPathSubsystem::DrawDebugGraph(FLinearColor Color, float Thickness, float Duration) const
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    const FColor DrawColor = Color.ToFColor(true);
    for (const FIFCPathEdge& Edge : Edges)
    {
        const FIFCPathNode* A = Nodes.Find(Edge.A);
        const FIFCPathNode* B = Nodes.Find(Edge.B);
        if (A == nullptr || B == nullptr)
        {
            continue;
        }

        const bool bPortalBlocked = !Edge.PortalId.IsEmpty() && BlockedPortals.Contains(Edge.PortalId);
        const bool bSpaceBlocked = (!A->SpaceId.IsEmpty() && BlockedSpaces.Contains(A->SpaceId))
            || (!B->SpaceId.IsEmpty() && BlockedSpaces.Contains(B->SpaceId));
        const bool bPenalized = GetTraversalMultiplier(*A, *B) > 1.0;

        FColor EdgeColor = DrawColor;
        if (bPortalBlocked || bSpaceBlocked)
        {
            EdgeColor = FColor::Red;
        }
        else if (bPenalized)
        {
            EdgeColor = FColor::Yellow;
        }

        DrawDebugLine(World, A->Position, B->Position, EdgeColor, false, Duration, 0, Thickness);
    }
}

void UIFCPathSubsystem::DrawDebugNavMesh(FLinearColor Color, float Thickness, float Duration) const
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    const FColor DrawColor = Color.ToFColor(true);
    for (const TPair<FString, FIFCPathCell>& Pair : Cells)
    {
        const FIFCPathCell& Cell = Pair.Value;
        if (Cell.Vertices.Num() != 3)
        {
            continue;
        }
        const FColor CellColor = BlockedSpaces.Contains(Cell.SpaceId) ? FColor::Red : DrawColor;
        DrawDebugLine(World, Cell.Vertices[0], Cell.Vertices[1], CellColor, false, Duration, 0, Thickness);
        DrawDebugLine(World, Cell.Vertices[1], Cell.Vertices[2], CellColor, false, Duration, 0, Thickness);
        DrawDebugLine(World, Cell.Vertices[2], Cell.Vertices[0], CellColor, false, Duration, 0, Thickness);
    }
}

double UIFCPathSubsystem::GetTraversalMultiplier(const FIFCPathNode& A, const FIFCPathNode& B) const
{
    double Multiplier = 1.0;
    if (!A.SpaceId.IsEmpty())
    {
        if (const double* Value = SpaceCostMultipliers.Find(A.SpaceId))
        {
            Multiplier = FMath::Max(Multiplier, *Value);
        }
    }
    if (!B.SpaceId.IsEmpty())
    {
        if (const double* Value = SpaceCostMultipliers.Find(B.SpaceId))
        {
            Multiplier = FMath::Max(Multiplier, *Value);
        }
    }
    return Multiplier;
}

const FIFCPathCell* UIFCPathSubsystem::FindCellAtWorldPosition(const FVector& WorldPosition) const
{
    const FIFCPathCell* BestCell = nullptr;
    double BestZDistance = TNumericLimits<double>::Max();

    for (const TPair<FString, FIFCPathCell>& Pair : Cells)
    {
        const FIFCPathCell& Cell = Pair.Value;
        if (Cell.Vertices.Num() != 3 || !PointInCell2D(WorldPosition, Cell))
        {
            continue;
        }

        const double CellZ = (Cell.Vertices[0].Z + Cell.Vertices[1].Z + Cell.Vertices[2].Z) / 3.0;
        const double ZDistance = FMath::Abs(WorldPosition.Z - CellZ);
        if (ZDistance <= CellPlaneToleranceCm && ZDistance < BestZDistance)
        {
            BestZDistance = ZDistance;
            BestCell = &Cell;
        }
    }

    return BestCell;
}

bool UIFCPathSubsystem::FindCellCorridor(
    const FString& StartCellId,
    const FString& GoalCellId,
    const FString& SpaceId,
    TArray<FString>& OutCellIds) const
{
    OutCellIds.Reset();
    if (!Cells.Contains(StartCellId) || !Cells.Contains(GoalCellId))
    {
        return false;
    }

    TMap<FString, double> Dist;
    TMap<FString, FString> Prev;
    TSet<FString> Unvisited;
    for (const TPair<FString, FIFCPathCell>& Pair : Cells)
    {
        if (Pair.Value.SpaceId == SpaceId)
        {
            Dist.Add(Pair.Key, Pair.Key == StartCellId ? 0.0 : TNumericLimits<double>::Max());
            Unvisited.Add(Pair.Key);
        }
    }

    while (Unvisited.Num() > 0)
    {
        FString Current;
        double Best = TNumericLimits<double>::Max();
        for (const FString& Candidate : Unvisited)
        {
            const double CandidateDist = Dist.FindRef(Candidate);
            if (CandidateDist < Best)
            {
                Best = CandidateDist;
                Current = Candidate;
            }
        }
        if (Current.IsEmpty() || Best == TNumericLimits<double>::Max())
        {
            break;
        }
        if (Current == GoalCellId)
        {
            break;
        }
        Unvisited.Remove(Current);

        const FIFCPathCell* CurrentCell = Cells.Find(Current);
        if (CurrentCell == nullptr)
        {
            continue;
        }
        const FVector CurrentCentroid = CellCentroid(*CurrentCell);
        for (const FString& NeighbourId : CurrentCell->NeighborIds)
        {
            if (!Unvisited.Contains(NeighbourId))
            {
                continue;
            }
            const FIFCPathCell* Neighbour = Cells.Find(NeighbourId);
            if (Neighbour == nullptr || Neighbour->SpaceId != SpaceId)
            {
                continue;
            }
            const double CandidateDist = Best + FVector::Dist2D(CurrentCentroid, CellCentroid(*Neighbour));
            if (CandidateDist < Dist.FindRef(NeighbourId))
            {
                Dist[NeighbourId] = CandidateDist;
                Prev.Add(NeighbourId, Current);
            }
        }
    }

    if (StartCellId != GoalCellId && !Prev.Contains(GoalCellId))
    {
        return false;
    }

    FString Cursor = GoalCellId;
    OutCellIds.Add(Cursor);
    while (Cursor != StartCellId)
    {
        const FString* Parent = Prev.Find(Cursor);
        if (Parent == nullptr)
        {
            OutCellIds.Reset();
            return false;
        }
        Cursor = *Parent;
        OutCellIds.Add(Cursor);
    }
    Algo::Reverse(OutCellIds);
    return OutCellIds.Num() > 0;
}

bool UIFCPathSubsystem::GetSharedCellEdge(
    const FIFCPathCell& A,
    const FIFCPathCell& B,
    FVector& OutA,
    FVector& OutB) const
{
    TArray<FVector> Common;
    for (const FVector& AV : A.Vertices)
    {
        for (const FVector& BV : B.Vertices)
        {
            if (NearlyEqual2D(AV, BV) && FMath::Abs(AV.Z - BV.Z) <= 0.01)
            {
                bool bAlreadyAdded = false;
                for (const FVector& Existing : Common)
                {
                    if (NearlyEqual2D(Existing, AV))
                    {
                        bAlreadyAdded = true;
                        break;
                    }
                }
                if (!bAlreadyAdded)
                {
                    Common.Add(AV);
                }
                break;
            }
        }
    }

    if (Common.Num() != 2)
    {
        return false;
    }
    OutA = Common[0];
    OutB = Common[1];
    return true;
}

TPair<FVector, FVector> UIFCPathSubsystem::OrientPortal(
    const FIFCPathCell& Current,
    const FIFCPathCell& Next,
    const FVector& A,
    const FVector& B)
{
    const FVector CurrentCenter = CellCentroid(Current);
    const FVector NextCenter = CellCentroid(Next);
    const double CrossA = SignedArea2D(CurrentCenter, NextCenter, A);
    const double CrossB = SignedArea2D(CurrentCenter, NextCenter, B);

    // Match the same left/right winding convention as the Python reference
    // implementation. SignedArea2D compensates for IFC->Unreal Y mirroring.
    return CrossA >= CrossB
        ? TPair<FVector, FVector>(B, A)
        : TPair<FVector, FVector>(A, B);
}

void UIFCPathSubsystem::StringPull(
    const TArray<TPair<FVector, FVector>>& Portals,
    TArray<FVector>& OutPoints)
{
    OutPoints.Reset();
    if (Portals.Num() == 0)
    {
        return;
    }

    FVector Apex = Portals[0].Key;
    FVector Left = Portals[0].Key;
    FVector Right = Portals[0].Value;
    int32 ApexIndex = 0;
    int32 LeftIndex = 0;
    int32 RightIndex = 0;
    OutPoints.Add(Apex);

    int32 Index = 1;
    while (Index < Portals.Num())
    {
        const FVector NewLeft = Portals[Index].Key;
        const FVector NewRight = Portals[Index].Value;

        if (SignedArea2D(Apex, Right, NewRight) <= FunnelEpsilon)
        {
            if (NearlyEqual2D(Apex, Right)
                || SignedArea2D(Apex, Left, NewRight) > FunnelEpsilon)
            {
                Right = NewRight;
                RightIndex = Index;
            }
            else
            {
                OutPoints.Add(Left);
                Apex = Left;
                ApexIndex = LeftIndex;
                Left = Apex;
                Right = Apex;
                LeftIndex = ApexIndex;
                RightIndex = ApexIndex;
                Index = ApexIndex + 1;
                continue;
            }
        }

        if (SignedArea2D(Apex, Left, NewLeft) >= -FunnelEpsilon)
        {
            if (NearlyEqual2D(Apex, Left)
                || SignedArea2D(Apex, Right, NewLeft) < -FunnelEpsilon)
            {
                Left = NewLeft;
                LeftIndex = Index;
            }
            else
            {
                OutPoints.Add(Right);
                Apex = Right;
                ApexIndex = RightIndex;
                Left = Apex;
                Right = Apex;
                LeftIndex = ApexIndex;
                RightIndex = ApexIndex;
                Index = ApexIndex + 1;
                continue;
            }
        }

        ++Index;
    }

    const FVector Goal = Portals.Last().Key;
    if (!NearlyEqual2D(OutPoints.Last(), Goal))
    {
        OutPoints.Add(Goal);
    }

    for (int32 I = OutPoints.Num() - 1; I > 0; --I)
    {
        if (NearlyEqual2D(OutPoints[I], OutPoints[I - 1]))
        {
            OutPoints.RemoveAt(I);
        }
    }
}

bool UIFCPathSubsystem::PointInCell2D(const FVector& Point, const FIFCPathCell& Cell)
{
    if (Cell.Vertices.Num() != 3)
    {
        return false;
    }

    const FVector& A = Cell.Vertices[0];
    const FVector& B = Cell.Vertices[1];
    const FVector& C = Cell.Vertices[2];
    const double Area = SignedArea2D(A, B, C);
    if (FMath::Abs(Area) <= FunnelEpsilon)
    {
        return false;
    }

    const double S1 = SignedArea2D(A, B, Point);
    const double S2 = SignedArea2D(B, C, Point);
    const double S3 = SignedArea2D(C, A, Point);
    if (Area > 0.0)
    {
        return S1 >= -FunnelEpsilon && S2 >= -FunnelEpsilon && S3 >= -FunnelEpsilon;
    }
    return S1 <= FunnelEpsilon && S2 <= FunnelEpsilon && S3 <= FunnelEpsilon;
}

FVector UIFCPathSubsystem::CellCentroid(const FIFCPathCell& Cell)
{
    if (Cell.Vertices.Num() != 3)
    {
        return FVector::ZeroVector;
    }
    return (Cell.Vertices[0] + Cell.Vertices[1] + Cell.Vertices[2]) / 3.0;
}

double UIFCPathSubsystem::SignedArea2D(const FVector& A, const FVector& B, const FVector& C)
{
    // INAV is right-handed. ToUnrealPosition mirrors Y, which flips the sign of
    // every XY cross product. Negating Unreal's native signed area restores the
    // original INAV orientation so the C++ funnel matches the tested Python
    // reference algorithm exactly.
    return -((B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X));
}

bool UIFCPathSubsystem::NearlyEqual2D(const FVector& A, const FVector& B, double ToleranceCm)
{
    return FMath::Square(A.X - B.X) + FMath::Square(A.Y - B.Y)
        <= FMath::Square(ToleranceCm);
}

FVector UIFCPathSubsystem::ToUnrealPosition(double X, double Y, double Z)
{
    // INAV is right-handed Z-up in metres. Unreal is centimetres; mirroring Y changes handedness.
    return FVector(X * 100.0, -Y * 100.0, Z * 100.0);
}
