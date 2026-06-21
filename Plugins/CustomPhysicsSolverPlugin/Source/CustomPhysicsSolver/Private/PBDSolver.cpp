#include "PBDSolver.h"

void FPBDSolver::Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges)
{
	Particles = InParticles;
	Edges     = InEdges;
	BrokenEdges.Init(false, Edges.Num());
}

void FPBDSolver::Step(float DeltaTime, const FSolverConfig& Config)
{
	const double T0 = FPlatformTime::Seconds();
	// --- Symplectic Euler predict ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f) continue;

		P.PrevPosition  = P.Position;
		P.Velocity     += Config.Gravity * DeltaTime;
		P.Velocity     *= Config.Damping;
		P.Position     += P.Velocity * DeltaTime;
	}

	// --- Gauss-Seidel distance constraint ---
	for (int32 Iter = 0; Iter < Config.Iterations; ++Iter)
	{
		for (int32 ei = 0; ei < Edges.Num(); ++ei)
		{
			if (BrokenEdges[ei]) continue;
			const FSolverEdge& E = Edges[ei];

			FParticle& PA = Particles[E.A];
			FParticle& PB = Particles[E.B];

			const float WSum = PA.InvMass + PB.InvMass;
			if (WSum <= 0.f) continue;

			FVector Delta = PB.Position - PA.Position;
			float   Dist  = Delta.Size();
			if (Dist < SMALL_NUMBER) continue;

			const float C = Dist - E.RestLength;

			if (E.BreakThreshold > 0.f && C > E.BreakThreshold)
			{
				BrokenEdges[ei] = true;
				continue;
			}

			const FVector Grad    = Delta / Dist;
			const float   dLambda = -C / WSum;
			PA.Position -= PA.InvMass * dLambda * Grad;
			PB.Position += PB.InvMass * dLambda * Grad;
		}
	}

	// --- Floor collision (position projection) ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f || P.Position.Z >= Config.FloorZ) continue;
		P.Position.Z = Config.FloorZ;
	}

	// --- Velocity update ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f) continue;
		P.Velocity = (P.Position - P.PrevPosition) / DeltaTime;

		if (P.Position.Z <= Config.FloorZ + KINDA_SMALL_NUMBER && P.Velocity.Z < 0.f)
		{
			P.Velocity.Z  *= -Config.Restitution;
			P.Velocity.X  *= (1.f - Config.FloorFriction);
			P.Velocity.Y  *= (1.f - Config.FloorFriction);
		}
	}

	Diagnostics.StepTimeMs = (float)((FPlatformTime::Seconds() - T0) * 1000.0);
}
