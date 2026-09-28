#include "IFCPathSubsystem.h"

#include "Algo/Reverse.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool UIFCPathSubsystem::LoadInav(const FString& FilePath, FString& Error)
{
    Nodes.Reset();
    Edges.Reset();
    BlockedPortals.Reset();

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
        const TArray<TSharedPtr<FJsonValue>>* P = nullptr;
        if (!Obj->TryGetArrayField(TEXT("position_m"), P) || P == nullptr || P->Num() < 3)
        {
            continue;
        }

        FIFCPathNode Node;
        Node.Id = Obj->GetStringField(TEXT("id"));
        Node.Position = ToUnrealPosition((*P)[0]->AsNumber(), (*P)[1]->AsNumber(), (*P)[2]->AsNumber());
        Nodes.Add(Node.Id, Node);
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
            Edge.A = Obj->GetStringField(TEXT("a"));
            Edge.B = Obj->GetStringField(TEXT("b"));
            Edge.DistanceMeters = Obj->GetNumberField(TEXT("distance_m"));
            Obj->TryGetStringField(TEXT("portal_id"), Edge.PortalId);
            Edges.Add(MoveTemp(Edge));
        }
    }

    Error.Reset();
    return Nodes.Num() > 0;
}

bool UIFCPathSubsystem::FindPath(const FString& StartNodeId, const FString& GoalNodeId, TArray<FVector>& OutPoints) const
{
    OutPoints.Reset();
    if (!Nodes.Contains(StartNodeId) || !Nodes.Contains(GoalNodeId))
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

        for (const FIFCPathEdge& Edge : Edges)
        {
            if (!Edge.PortalId.IsEmpty() && BlockedPortals.Contains(Edge.PortalId))
            {
                continue;
            }

            FString Next;
            if (Edge.A == Current)
            {
                Next = Edge.B;
            }
            else if (Edge.B == Current)
            {
                Next = Edge.A;
            }
            else
            {
                continue;
            }

            if (!Unvisited.Contains(Next))
            {
                continue;
            }

            const double CandidateDist = Best + Edge.DistanceMeters;
            if (CandidateDist < Dist.FindRef(Next))
            {
                Dist[Next] = CandidateDist;
                Prev.Add(Next, Current);
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

FVector UIFCPathSubsystem::ToUnrealPosition(double X, double Y, double Z)
{
    // INAV is right-handed Z-up in metres. Unreal is centimetres; mirroring Y changes handedness.
    return FVector(X * 100.0, -Y * 100.0, Z * 100.0);
}
