#include "IFCPathSubsystem.h"

#include "Algo/Reverse.h"
#include "Dom/JsonObject.h"
#include "DrawDebugHelpers.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool UIFCPathSubsystem::LoadInav(const FString& FilePath, FString& Error)
{
    Nodes.Reset();
    Edges.Reset();
    Adjacency.Reset();
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

            // A blocked space is a no-entry region. An agent that starts inside
            // one may continue moving within that same start space to escape.
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

FVector UIFCPathSubsystem::ToUnrealPosition(double X, double Y, double Z)
{
    // INAV is right-handed Z-up in metres. Unreal is centimetres; mirroring Y changes handedness.
    return FVector(X * 100.0, -Y * 100.0, Z * 100.0);
}
