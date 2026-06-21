#include "AVBDSolver.h"

void FAVBDSolver::Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges)
{
	Particles = InParticles;
	Edges     = InEdges;
	Lambda.Init(0.f, Edges.Num());
	BrokenEdges.Init(false, Edges.Num());
	BuildGraphColoring();
	ensure(ValidateColoring());
}

void FAVBDSolver::Step(float DeltaTime, const FSolverConfig& Config)
{
	const double T0  = FPlatformTime::Seconds();
	const float h   = DeltaTime;
	const float h2  = h * h;
	const float Rho = Config.AugLagrangianRho;

	// --- 1. Save x_prev ---
	TArray<FVector> XPrev;
	XPrev.SetNum(Particles.Num());
	for (int32 i = 0; i < Particles.Num(); ++i)
		XPrev[i] = Particles[i].Position;

	// --- 2. Predict: x ← x + h·v + h²·g ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f) continue;
		P.Position += P.Velocity * h + Config.Gravity * h2;
	}

	// --- 3. Primal solve (λ FIXED during this loop) ---
	//
	//  Constraint:   C_c = |x_B - x_A| - L_c
	//  AL target:    drive C_c → -λ_c/ρ  (converges to 0 as λ→0)
	//
	//  Position correction per constraint:
	//    corr = -(C + λ/ρ) / (w_A + w_B)
	//    x_A -= w_A · corr · n
	//    x_B += w_B · corr · n
	//
	//  λ is NOT updated inside this loop.
	//  Separating primal/dual prevents Lambda from drifting to -ρ·C,
	//  which would zero out corrections and cause residual growth.
	for (int32 Iter = 0; Iter < Config.Iterations; ++Iter)
	{
		for (int32 Color = 0; Color < NumColors; ++Color)
		{
			for (int32 ei = 0; ei < Edges.Num(); ++ei)
			{
				if (EdgeColors[ei] != Color) continue;
				if (BrokenEdges[ei]) continue;

				const FSolverEdge& E  = Edges[ei];
				FParticle&         PA = Particles[E.A];
				FParticle&         PB = Particles[E.B];

				const float wA   = PA.InvMass;
				const float wB   = PB.InvMass;
				const float wSum = wA + wB;
				if (wSum <= 0.f) continue;

				const FVector d    = PB.Position - PA.Position;
				const float   dist = d.Size();
				if (dist < SMALL_NUMBER) continue;

				const FVector n = d / dist;
				const float   C = dist - E.RestLength;

				if (E.BreakThreshold > 0.f && C > E.BreakThreshold)
				{
					BrokenEdges[ei] = true;
					continue;
				}

				const float corr = -(C + Lambda[ei] / Rho) / wSum;
				PA.Position -= wA * corr * n;
				PB.Position += wB * corr * n;
			}
		}
	}

	// --- 4. Dual update: λ_c += ρ · C_c(x*)
	//  x* here is the primal-solved position (before floor projection).
	//  Updating λ after the primal loop (not inside it) is the Primal-Dual split
	//  that makes AVBD distinct from PBD.
	for (int32 ei = 0; ei < Edges.Num(); ++ei)
	{
		if (BrokenEdges[ei]) continue;
		const FSolverEdge& E = Edges[ei];
		const float C = (Particles[E.B].Position - Particles[E.A].Position).Size() - E.RestLength;
		Lambda[ei] += Config.AugLagrangianRho * C;
	}

	// --- 5. Floor collision (position projection) ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f || P.Position.Z >= Config.FloorZ) continue;
		P.Position.Z = Config.FloorZ;
	}

	// --- 6. Velocity update ---
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		FParticle& P = Particles[i];
		if (P.InvMass <= 0.f) continue;
		P.Velocity    = (P.Position - XPrev[i]) / h;
		P.Velocity   *= Config.Damping;

		if (P.Position.Z <= Config.FloorZ + KINDA_SMALL_NUMBER && P.Velocity.Z < 0.f)
		{
			P.Velocity.Z *= -Config.Restitution;
			P.Velocity.X *= (1.f - Config.FloorFriction);
			P.Velocity.Y *= (1.f - Config.FloorFriction);
		}

		P.PrevPosition = P.Position;
	}

	ComputeDiagnostics(h, Config);
	Diagnostics.StepTimeMs = (float)((FPlatformTime::Seconds() - T0) * 1000.0);
}

void FAVBDSolver::ComputeDiagnostics(float h, const FSolverConfig& Config)
{
	Diagnostics = FSolverDiagnostics{};

	for (const FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f) continue;
		const float mass = 1.f / P.InvMass;
		const float KE   = 0.5f * mass * P.Velocity.SizeSquared();
		const float PE   = mass * FMath::Abs(Config.Gravity.Z) * P.Position.Z;
		if (FMath::IsFinite(KE)) Diagnostics.KineticEnergy   += KE;
		if (FMath::IsFinite(PE)) Diagnostics.PotentialEnergy += PE;
	}

	float SumResidual = 0.f;
	int32 FiniteCount = 0;
	for (const FSolverEdge& E : Edges)
	{
		const float C = (Particles[E.B].Position - Particles[E.A].Position).Size() - E.RestLength;
		if (!FMath::IsFinite(C)) continue;
		const float AbsC = FMath::Abs(C);
		Diagnostics.MaxConstraintResidual = FMath::Max(Diagnostics.MaxConstraintResidual, AbsC);
		SumResidual += AbsC;
		++FiniteCount;
	}
	if (FiniteCount > 0)
		Diagnostics.AvgConstraintResidual = SumResidual / FiniteCount;
}
