#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "SolverTypes.h"
#include "PhysicsSolverComponent.generated.h"

class IPhysicsSolver;

UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class CUSTOMPHYSICSSOLVER_API UPhysicsSolverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPhysicsSolverComponent();

	// Blueprint で切り替えるソルバー種別
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver")
	ESolverType SolverType = ESolverType::PBD;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver")
	FSolverConfig Config;

	// Output Log に診断情報を出力する間隔（秒, 0 = 無効）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Diagnostics")
	float DiagLogInterval = 1.f;

	// ISM rendering
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Rendering")
	UStaticMesh* ParticleMesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Rendering")
	float ParticleScale = 10.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Rendering")
	bool bEnableISMRendering = true;

	// World-space offset applied to all particles at initialization
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver")
	FVector PositionOffset = FVector::ZeroVector;

	// Grid dimensions for tower layout
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Grid", meta=(ClampMin=1))
	int32 GridNx = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Grid", meta=(ClampMin=1))
	int32 GridNy = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Grid", meta=(ClampMin=1))
	int32 GridNz = 8;

	// Distance between adjacent particles (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Grid", meta=(ClampMin=1.0))
	float ParticleSpacing = 50.f;

	// Edge breaks when stretched beyond RestLength * this ratio (0 = never)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics Solver|Grid", meta=(ClampMin=0.0))
	float EdgeBreakRatio = 0.5f;

	// ソルバーを選択した SolverType で再構築する
	UFUNCTION(BlueprintCallable, Category="Physics Solver")
	void RebuildSolver();

	// 全自由粒子に速度を加算する（崩壊トリガー用）
	UFUNCTION(BlueprintCallable, Category="Physics Solver")
	void ApplyImpulseToAll(FVector Impulse);

	// 現在のパーティクル状態を返す（ISM 更新用）
	UFUNCTION(BlueprintCallable, Category="Physics Solver")
	TArray<FParticle> GetParticles() const;

	// 最新ステップの診断値を返す（全ソルバーで有効）
	UFUNCTION(BlueprintCallable, Category="Physics Solver|Diagnostics")
	FSolverDiagnostics GetDiagnostics() const;

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

private:
	TSharedPtr<IPhysicsSolver> Solver;

	UPROPERTY()
	UInstancedStaticMeshComponent* ISMComp = nullptr;

	void SpawnDefaultParticles();
	void UpdateISMInstances();

	TArray<FParticle>   InitParticles;
	TArray<FSolverEdge> InitEdges;

	float DiagAccum = 0.f;
};
