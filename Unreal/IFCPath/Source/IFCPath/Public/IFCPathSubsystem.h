#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "IFCPathSubsystem.generated.h"

USTRUCT(BlueprintType)
struct FIFCPathNode
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FString Id;

    UPROPERTY(BlueprintReadOnly)
    FVector Position = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly)
    FString Kind;

    UPROPERTY(BlueprintReadOnly)
    FString LevelId;

    UPROPERTY(BlueprintReadOnly)
    FString SpaceId;

    UPROPERTY(BlueprintReadOnly)
    FString PortalId;
};

USTRUCT()
struct FIFCPathEdge
{
    GENERATED_BODY()

    FString A;
    FString B;
    double DistanceMeters = 0.0;
    FString PortalId;
};

struct FIFCPathAdjacencyEntry
{
    FString NodeId;
    double DistanceMeters = 0.0;
    FString PortalId;
};

UCLASS()
class IFCPATH_API UIFCPathSubsystem : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category="IFCPath")
    bool LoadInav(const FString& FilePath, FString& Error);

    UFUNCTION(BlueprintCallable, Category="IFCPath")
    bool FindPath(const FString& StartNodeId, const FString& GoalNodeId, TArray<FVector>& OutPoints) const;

    UFUNCTION(BlueprintCallable, Category="IFCPath")
    bool FindNearestNode(const FVector& WorldPosition, float MaxDistanceCm, FString& OutNodeId, FVector& OutNodePosition) const;

    UFUNCTION(BlueprintCallable, Category="IFCPath")
    bool FindPathFromWorldPositions(
        const FVector& StartWorldPosition,
        const FVector& GoalWorldPosition,
        float MaxSnapDistanceCm,
        TArray<FVector>& OutPoints,
        FString& OutStartNodeId,
        FString& OutGoalNodeId) const;

    UFUNCTION(BlueprintCallable, Category="IFCPath|Dynamic State")
    void SetPortalBlocked(const FString& PortalId, bool bBlocked);

    UFUNCTION(BlueprintCallable, Category="IFCPath|Dynamic State")
    void SetSpaceBlocked(const FString& SpaceId, bool bBlocked);

    UFUNCTION(BlueprintCallable, Category="IFCPath|Dynamic State")
    void SetSpaceCostMultiplier(const FString& SpaceId, float CostMultiplier);

    UFUNCTION(BlueprintCallable, Category="IFCPath|Dynamic State")
    void ClearDynamicState();

    UFUNCTION(BlueprintPure, Category="IFCPath|Dynamic State")
    bool IsSpaceBlocked(const FString& SpaceId) const;

    UFUNCTION(BlueprintPure, Category="IFCPath|Dynamic State")
    float GetSpaceCostMultiplier(const FString& SpaceId) const;

    UFUNCTION(BlueprintCallable, Category="IFCPath|Debug")
    void DrawDebugPath(const TArray<FVector>& Points, FLinearColor Color, float Thickness = 8.0f, float Duration = 10.0f) const;

    UFUNCTION(BlueprintCallable, Category="IFCPath|Debug")
    void DrawDebugGraph(FLinearColor Color, float Thickness = 1.5f, float Duration = 10.0f) const;

    UFUNCTION(BlueprintPure, Category="IFCPath")
    int32 GetNodeCount() const { return Nodes.Num(); }

    UFUNCTION(BlueprintPure, Category="IFCPath")
    int32 GetEdgeCount() const { return Edges.Num(); }

private:
    TMap<FString, FIFCPathNode> Nodes;
    TArray<FIFCPathEdge> Edges;
    TMap<FString, TArray<FIFCPathAdjacencyEntry>> Adjacency;

    TSet<FString> BlockedPortals;
    TSet<FString> BlockedSpaces;
    TMap<FString, double> SpaceCostMultipliers;

    double GetTraversalMultiplier(const FIFCPathNode& A, const FIFCPathNode& B) const;
    static FVector ToUnrealPosition(double X, double Y, double Z);
};
