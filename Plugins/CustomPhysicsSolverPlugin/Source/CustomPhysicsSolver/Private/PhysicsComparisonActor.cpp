#include "PhysicsComparisonActor.h"
#include "PhysicsSolverComponent.h"
#include "ChaosStackActor.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"

APhysicsComparisonActor::APhysicsComparisonActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	SolverLeft  = CreateDefaultSubobject<UPhysicsSolverComponent>(TEXT("SolverLeft"));
	SolverRight = CreateDefaultSubobject<UPhysicsSolverComponent>(TEXT("SolverRight"));

	SolverLeft->SolverType  = ESolverType::AVBD_GPU;
	SolverRight->SolverType = ESolverType::PBD;

	SolverLeft->DiagLogInterval  = 0.f;
	SolverRight->DiagLogInterval = 0.f;

	// ParticleScale: sphere diameter = ParticleScale * 100cm (UE sphere mesh = 100cm diameter)
	// 50cm spacing → 0.4 gives 40cm spheres (slightly overlapping = solid look)
	SolverLeft->ParticleScale  = 0.4f;
	SolverRight->ParticleScale = 0.4f;
}

void APhysicsComparisonActor::BeginPlay()
{
	const float Half = ChainSeparation * 0.5f;

	SolverLeft->PositionOffset  = FVector(-Half, 0.f, 0.f);
	SolverRight->PositionOffset = FVector( Half, 0.f, 0.f);

	if (ParticleMesh)
	{
		SolverLeft->ParticleMesh  = ParticleMesh;
		SolverRight->ParticleMesh = ParticleMesh;
		SolverLeft->bEnableISMRendering  = true;
		SolverRight->bEnableISMRendering = true;
	}

	Super::BeginPlay();

	// Bind Space key to TriggerCollapse
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		EnableInput(PC);
		InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this,
			&APhysicsComparisonActor::TriggerCollapse);
	}
}

void APhysicsComparisonActor::TriggerCollapse()
{
	if (SolverLeft)  SolverLeft->ApplyImpulseToAll( CollapseImpulse);
	if (SolverRight) SolverRight->ApplyImpulseToAll(CollapseImpulse);

	for (AChaosStackActor* Chaos : ChaosActors)
		if (Chaos) Chaos->TriggerCollapse(CollapseImpulse);

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(100, 2.f, FColor::Yellow, TEXT("Collapse triggered! [Space]"));
}

void APhysicsComparisonActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!GEngine) return;

	auto Print = [](int32 Key, const FColor& Col, const TCHAR* Label, const FSolverDiagnostics& D)
	{
		GEngine->AddOnScreenDebugMessage(Key, 0.f, Col,
			FString::Printf(TEXT("[%s]  Step: %.3f ms  |  MaxRes: %.4f cm  |  KE: %.2f"),
				Label, D.StepTimeMs, D.MaxConstraintResidual, D.KineticEnergy));
	};

	if (SolverLeft)
		Print(101, FColor(100, 200, 255), TEXT("AVBD GPU"), SolverLeft->GetDiagnostics());
	if (SolverRight)
		Print(102, FColor(255, 180,  80), TEXT("PBD CPU "), SolverRight->GetDiagnostics());

	int32 TotalChaosBlocks = 0;
	for (AChaosStackActor* Chaos : ChaosActors)
		if (Chaos) TotalChaosBlocks += Chaos->GetBlockCount();
	if (TotalChaosBlocks > 0)
		GEngine->AddOnScreenDebugMessage(103, 0.f, FColor(100, 255, 100),
			FString::Printf(TEXT("[Chaos   ]  Blocks: %d  (Space = trigger collapse)"), TotalChaosBlocks));
}
