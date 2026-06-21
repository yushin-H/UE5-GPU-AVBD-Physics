#pragma once

#include "CoreMinimal.h"
#include "IPhysicsSolver.h"

class CUSTOMPHYSICSSOLVER_API FPBDSolver : public IPhysicsSolver
{
public:
	virtual void Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges) override;
	virtual void Step(float DeltaTime, const FSolverConfig& Config) override;
	virtual const TArray<FParticle>& GetParticles() const override { return Particles; }
	virtual ESolverType GetType() const override { return ESolverType::PBD; }
	virtual FSolverDiagnostics GetDiagnostics() const override { return Diagnostics; }
	virtual void ApplyImpulseToAll(const FVector& Impulse) override
	{
		for (FParticle& P : Particles)
			if (P.InvMass > 0.f) P.Velocity += Impulse;
	}

private:
	TArray<FParticle>   Particles;
	TArray<FSolverEdge> Edges;
	TArray<bool>        BrokenEdges;
	FSolverDiagnostics  Diagnostics;
};
