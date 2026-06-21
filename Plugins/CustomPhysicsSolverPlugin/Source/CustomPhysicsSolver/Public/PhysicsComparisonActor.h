#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SolverTypes.h"
#include "PhysicsComparisonActor.generated.h"

class UPhysicsSolverComponent;
class AChaosStackActor;

UCLASS()
class CUSTOMPHYSICSSOLVER_API APhysicsComparisonActor : public AActor
{
	GENERATED_BODY()

public:
	APhysicsComparisonActor();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Comparison")
	float ChainSeparation = 600.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Comparison")
	UStaticMesh* ParticleMesh = nullptr;

	// Velocity impulse applied to all particles when Space is pressed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Comparison")
	FVector CollapseImpulse = FVector(300.f, 0.f, 0.f);

	// Drag Chaos stack actor(s) here in the editor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Comparison")
	TArray<AChaosStackActor*> ChaosActors;

	UFUNCTION(BlueprintCallable, Category="Comparison")
	void TriggerCollapse();

private:
	UPROPERTY(VisibleAnywhere)
	UPhysicsSolverComponent* SolverLeft = nullptr;   // AVBD GPU

	UPROPERTY(VisibleAnywhere)
	UPhysicsSolverComponent* SolverRight = nullptr;  // PBD
};
