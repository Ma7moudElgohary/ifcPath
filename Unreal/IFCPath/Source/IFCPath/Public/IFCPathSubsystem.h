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

    UPROPERTY(BlueprintReadOnly)
    FString CellId;
};

USTRUCT(BlueprintType)
struct FIFCPathCell
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FString Id;

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> Vertices;

    UPROPERTY(BlueprintReadOnly)
    FString SpaceId;

    UPROPERTY(BlueprintReadOnly)
    FString LevelId;

    UPROPERTY(BlueprintReadOnly)
    TArray<FString> NeighborIds;
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

struct FIFCPathHierarchyAnchor
{
    FString Id;
    FString SpaceId;
    FVector Position = FVector::ZeroVector;
};

struct FIFCPathHierarchyTransfer
{
    FString Id;
    FString Kind;
    FString PortalId;
    FString FromSpaceId;
    FString ToSpaceId;
    FVector FromPoint = FVector::ZeroVector;
    FVector ToPoint = FVector::ZeroVector;
    TArray<FVector> Points;
    bool bBidirectional = true;
};

struct FIFCPathHierarchyGraphEdge
{
    FString TargetId;
    double CostCm = 0.0;
    FString Kind;
    FString FromSpaceId;
    FString ToSpaceId;
    FString TransitionId;
    FString PortalId;
    TArray<FVector> Points;
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

    UFUNCTION(BlueprintCallable, Category="IFCPath|NavMesh")
    bool FindNavMeshPathFromWorldPositions(
        const FVector& StartWorldPosition,
        const FVector& GoalWorldPosition,
        TArray<FVector>& OutPoints,
        FString& OutSpaceId) const;

    // Building-wide route: exact funnel geometry inside spaces plus semantic
    // door/stair/ramp transfers between spaces and storeys. Dynamic blocked
    // portals/spaces and space cost multipliers are applied during route search.
    UFUNCTION(BlueprintCallable, Category="IFCPath|Hierarchical")
    bool FindHierarchicalPathFromWorldPositions(
        const FVector& StartWorldPosition,
        const FVector& GoalWorldPosition,
        TArray<FVector>& OutPoints,
        TArray<FString>& OutSpaceIds,
        TArray<FString>& OutTransitionIds,
        float& OutLengthMeters,
        float& OutWeightedCostMeters) const;

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

    UFUNCTION(BlueprintCallable, Category="IFCPath|Debug")
    void DrawDebugNavMesh(FLinearColor Color, float Thickness = 1.0f, float Duration = 10.0f) const;

    UFUNCTION(BlueprintPure, Category="IFCPath")
    int32 GetNodeCount() const { return Nodes.Num(); }

    UFUNCTION(BlueprintPure, Category="IFCPath")
    int32 GetEdgeCount() const { return Edges.Num(); }

    UFUNCTION(BlueprintPure, Category="IFCPath|NavMesh")
    int32 GetCellCount() const { return Cells.Num(); }

private:
    TMap<FString, FIFCPathNode> Nodes;
    TArray<FIFCPathEdge> Edges;
    TMap<FString, TArray<FIFCPathAdjacencyEntry>> Adjacency;
    TMap<FString, FIFCPathCell> Cells;

    TSet<FString> BlockedPortals;
    TSet<FString> BlockedSpaces;
    TMap<FString, double> SpaceCostMultipliers;

    double GetTraversalMultiplier(const FIFCPathNode& A, const FIFCPathNode& B) const;

    const FIFCPathCell* FindCellAtWorldPosition(const FVector& WorldPosition) const;
    const FIFCPathCell* FindCellAtWorldPositionInSpace(
        const FVector& WorldPosition,
        const FString& SpaceId) const;
    bool SnapPointToSpaceNavMesh(
        const FVector& WorldPosition,
        const FString& SpaceId,
        double MaxDistanceCm,
        FVector& OutPoint) const;
    bool FindNavMeshPathInSpace(
        const FVector& StartWorldPosition,
        const FVector& GoalWorldPosition,
        const FString& SpaceId,
        TArray<FVector>& OutPoints) const;
    bool FindLocalPathInSpace(
        const FVector& StartWorldPosition,
        const FVector& GoalWorldPosition,
        const FString& SpaceId,
        double MaxSnapDistanceCm,
        TArray<FVector>& OutPoints) const;
    void BuildHierarchyTransfers(TArray<FIFCPathHierarchyTransfer>& OutTransfers) const;
    bool FindVerticalTransferPath(
        const TSet<FString>& Component,
        const FString& FromLandingId,
        const FString& ToLandingId,
        TArray<FVector>& OutPoints) const;

    bool FindCellCorridor(
        const FString& StartCellId,
        const FString& GoalCellId,
        const FString& SpaceId,
        TArray<FString>& OutCellIds) const;
    bool GetSharedCellEdge(
        const FIFCPathCell& A,
        const FIFCPathCell& B,
        FVector& OutA,
        FVector& OutB) const;
    static TPair<FVector, FVector> OrientPortal(
        const FIFCPathCell& Current,
        const FIFCPathCell& Next,
        const FVector& A,
        const FVector& B);
    static void StringPull(
        const TArray<TPair<FVector, FVector>>& Portals,
        TArray<FVector>& OutPoints);
    static bool PointInCell2D(const FVector& Point, const FIFCPathCell& Cell);
    static FVector CellCentroid(const FIFCPathCell& Cell);
    static FVector ClosestPointOnSegment2D(const FVector& Point, const FVector& A, const FVector& B);
    static double PolylineLengthCm(const TArray<FVector>& Points);
    static void AppendUniquePoints(TArray<FVector>& Target, const TArray<FVector>& Source);
    static double SignedArea2D(const FVector& A, const FVector& B, const FVector& C);
    static bool NearlyEqual2D(const FVector& A, const FVector& B, double ToleranceCm = 0.01);

    static FVector ToUnrealPosition(double X, double Y, double Z);
};
