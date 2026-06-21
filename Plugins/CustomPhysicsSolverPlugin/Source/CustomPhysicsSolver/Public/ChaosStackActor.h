#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ChaosStackActor.generated.h"

UCLASS()
class CUSTOMPHYSICSSOLVER_API AChaosStackActor : public AActor
{
	GENERATED_BODY()

public:
	AChaosStackActor();

	virtual void BeginPlay() override;

	// Mesh used for each block (assign a Cube mesh in editor)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Chaos Stack")
	UStaticMesh* BlockMesh = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Chaos Stack", meta=(ClampMin=1))
	int32 GridNx = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Chaos Stack", meta=(ClampMin=1))
	int32 GridNy = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Chaos Stack", meta=(ClampMin=1))
	int32 GridNz = 8;

	// Center-to-center spacing between blocks (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Chaos Stack")
	float BlockSize = 50.f;

	// Enable simulate physics and add impulse to all blocks
	UFUNCTION(BlueprintCallable, Category="Chaos Stack")
	void TriggerCollapse(FVector Impulse);

	int32 GetBlockCount() const { return PhysicsBlocks.Num(); }

private:
	UPROPERTY()
	TArray<UStaticMeshComponent*> PhysicsBlocks;
};
