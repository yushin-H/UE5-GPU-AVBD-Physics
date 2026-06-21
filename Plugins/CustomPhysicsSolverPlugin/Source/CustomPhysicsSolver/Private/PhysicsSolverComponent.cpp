#include "PhysicsSolverComponent.h"
#include "PBDSolver.h"
#include "AVBDSolver.h"
#include "AVBDSolverGPU.h"

UPhysicsSolverComponent::UPhysicsSolverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UPhysicsSolverComponent::BeginPlay()
{
	Super::BeginPlay();
	SpawnDefaultParticles();
	RebuildSolver();

	if (bEnableISMRendering && ParticleMesh && GetOwner())
	{
		ISMComp = NewObject<UInstancedStaticMeshComponent>(GetOwner(),
			MakeUniqueObjectName(GetOwner(), UInstancedStaticMeshComponent::StaticClass(), TEXT("ParticleISM")));
		ISMComp->SetupAttachment(GetOwner()->GetRootComponent());
		ISMComp->SetStaticMesh(ParticleMesh);
		ISMComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		ISMComp->RegisterComponent();
		GetOwner()->AddInstanceComponent(ISMComp);

		const FVector Scale3D(ParticleScale, ParticleScale, ParticleScale);
		const int32 N = InitParticles.Num();
		for (int32 i = 0; i < N; ++i)
			ISMComp->AddInstance(FTransform(FQuat::Identity, InitParticles[i].Position, Scale3D));
	}
}

void UPhysicsSolverComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                             FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Solver) return;

	Solver->Step(DeltaTime, Config);

	if (ISMComp)
		UpdateISMInstances();

	const bool bIsAVBD = (SolverType == ESolverType::AVBD || SolverType == ESolverType::AVBD_GPU);
	if (DiagLogInterval > 0.f && bIsAVBD)
	{
		DiagAccum += DeltaTime;
		if (DiagAccum >= DiagLogInterval)
		{
			DiagAccum = 0.f;
			const FSolverDiagnostics D = GetDiagnostics();
			const TCHAR* Tag = (SolverType == ESolverType::AVBD_GPU) ? TEXT("AVBD_GPU") : TEXT("AVBD");
			UE_LOG(LogTemp, Log,
				TEXT("[%s] KE=%.2f  PE=%.2f  MaxRes=%.4f cm  AvgRes=%.4f cm  Step=%.3f ms"),
				Tag, D.KineticEnergy, D.PotentialEnergy,
				D.MaxConstraintResidual, D.AvgConstraintResidual, D.StepTimeMs);
		}
	}
}

void UPhysicsSolverComponent::UpdateISMInstances()
{
	if (!ISMComp) return;
	const TArray<FParticle>& Parts = Solver->GetParticles();
	const int32 N = FMath::Min(Parts.Num(), ISMComp->GetInstanceCount());
	const FVector Scale3D(ParticleScale, ParticleScale, ParticleScale);
	for (int32 i = 0; i < N; ++i)
	{
		if (!Parts[i].Position.ContainsNaN())
			ISMComp->UpdateInstanceTransform(i,
				FTransform(FQuat::Identity, Parts[i].Position, Scale3D),
				false, (i == N - 1));
	}
}

void UPhysicsSolverComponent::ApplyImpulseToAll(FVector Impulse)
{
	if (Solver) Solver->ApplyImpulseToAll(Impulse);
}

void UPhysicsSolverComponent::RebuildSolver()
{
	switch (SolverType)
	{
		case ESolverType::AVBD:
			Solver = MakeShared<FAVBDSolver>();
			break;
		case ESolverType::AVBD_GPU:
			Solver = MakeShared<FAVBDSolverGPU>();
			break;
		default:
			Solver = MakeShared<FPBDSolver>();
			break;
	}
	Solver->Initialize(InitParticles, InitEdges);
	DiagAccum = 0.f;
}

TArray<FParticle> UPhysicsSolverComponent::GetParticles() const
{
	return Solver ? Solver->GetParticles() : TArray<FParticle>();
}

FSolverDiagnostics UPhysicsSolverComponent::GetDiagnostics() const
{
	return Solver ? Solver->GetDiagnostics() : FSolverDiagnostics{};
}

void UPhysicsSolverComponent::SpawnDefaultParticles()
{
	const int32 Nx = FMath::Max(GridNx, 1);
	const int32 Ny = FMath::Max(GridNy, 1);
	const int32 Nz = FMath::Max(GridNz, 1);
	const float S  = ParticleSpacing;

	InitParticles.SetNum(Nx * Ny * Nz);
	InitEdges.Reset();

	// Particle index helper
	auto Idx = [&](int32 x, int32 y, int32 z) { return x + Nx * (y + Ny * z); };

	// Center the base of the tower at PositionOffset, bottom layer starts at Z = S
	const float HalfX = (Nx - 1) * S * 0.5f;
	const float HalfY = (Ny - 1) * S * 0.5f;

	for (int32 z = 0; z < Nz; ++z)
	for (int32 y = 0; y < Ny; ++y)
	for (int32 x = 0; x < Nx; ++x)
	{
		FParticle& P  = InitParticles[Idx(x, y, z)];
		P.Position    = PositionOffset + FVector(x * S - HalfX, y * S - HalfY, z * S + S);
		P.PrevPosition = P.Position;
		P.Velocity    = FVector::ZeroVector;
		P.InvMass     = 1.f;
	}

	// Add an edge between two particles (computes RestLength from current positions)
	auto AddEdge = [&](int32 A, int32 B)
	{
		FSolverEdge E;
		E.A          = A;
		E.B          = B;
		E.RestLength      = (InitParticles[A].Position - InitParticles[B].Position).Size();
		E.Stiffness       = 1e4f;
		E.BreakThreshold  = (EdgeBreakRatio > 0.f) ? E.RestLength * EdgeBreakRatio : 0.f;
		InitEdges.Add(E);
	};

	for (int32 z = 0; z < Nz; ++z)
	for (int32 y = 0; y < Ny; ++y)
	for (int32 x = 0; x < Nx; ++x)
	{
		// Axis-aligned structural edges
		if (x + 1 < Nx) AddEdge(Idx(x,y,z), Idx(x+1,y,z));
		if (y + 1 < Ny) AddEdge(Idx(x,y,z), Idx(x,y+1,z));
		if (z + 1 < Nz) AddEdge(Idx(x,y,z), Idx(x,y,z+1));

		// Diagonal bracing in vertical faces (prevents shear collapse)
		if (x + 1 < Nx && z + 1 < Nz) AddEdge(Idx(x,y,z), Idx(x+1,y,z+1));
		if (y + 1 < Ny && z + 1 < Nz) AddEdge(Idx(x,y,z), Idx(x,y+1,z+1));
	}
}
