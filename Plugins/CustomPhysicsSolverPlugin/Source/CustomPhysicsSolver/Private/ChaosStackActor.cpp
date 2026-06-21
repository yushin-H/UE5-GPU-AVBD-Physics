#include "ChaosStackActor.h"
#include "Components/StaticMeshComponent.h"

AChaosStackActor::AChaosStackActor()
{
	PrimaryActorTick.bCanEverTick = false;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

void AChaosStackActor::BeginPlay()
{
	Super::BeginPlay();

	if (!BlockMesh) return;

	const float HalfX   = (GridNx - 1) * BlockSize * 0.5f;
	const float HalfY   = (GridNy - 1) * BlockSize * 0.5f;
	const float Scale   = BlockSize * 0.9f / 100.f;  // 90% fill, UE cube = 100cm

	for (int32 z = 0; z < GridNz; ++z)
	for (int32 y = 0; y < GridNy; ++y)
	for (int32 x = 0; x < GridNx; ++x)
	{
		UStaticMeshComponent* Block = NewObject<UStaticMeshComponent>(this,
			MakeUniqueObjectName(this, UStaticMeshComponent::StaticClass(), TEXT("Block")));
		Block->SetupAttachment(GetRootComponent());
		Block->SetStaticMesh(BlockMesh);
		Block->SetRelativeLocation(FVector(
			x * BlockSize - HalfX,
			y * BlockSize - HalfY,
			z * BlockSize + BlockSize * 0.5f   // bottom of stack at Z=0
		));
		Block->SetRelativeScale3D(FVector(Scale));
		Block->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		Block->SetCollisionProfileName(TEXT("BlockAll"));
		Block->SetSimulatePhysics(false);  // stays static until TriggerCollapse
		Block->RegisterComponent();
		AddInstanceComponent(Block);
		PhysicsBlocks.Add(Block);
	}
}

void AChaosStackActor::TriggerCollapse(FVector Impulse)
{
	for (UStaticMeshComponent* Block : PhysicsBlocks)
	{
		if (!Block) continue;
		Block->SetSimulatePhysics(true);
		// bVelChange=true: treated as direct velocity change (mass-independent)
		Block->AddImpulse(Impulse, NAME_None, true);
	}
}
