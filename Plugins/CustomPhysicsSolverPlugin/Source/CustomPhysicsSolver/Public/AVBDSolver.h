#pragma once

#include "CoreMinimal.h"
#include "IPhysicsSolver.h"

class CUSTOMPHYSICSSOLVER_API FAVBDSolver : public IPhysicsSolver
{
public:
	virtual void Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges) override;
	virtual void Step(float DeltaTime, const FSolverConfig& Config) override;
	virtual const TArray<FParticle>& GetParticles() const override { return Particles; }
	virtual ESolverType GetType() const override { return ESolverType::AVBD; }

	bool ValidateColoring() const;

	// Call after Step() to read the diagnostics computed during that step
	virtual FSolverDiagnostics GetDiagnostics() const override { return Diagnostics; }
	virtual void ApplyImpulseToAll(const FVector& Impulse) override
	{
		for (FParticle& P : Particles)
			if (P.InvMass > 0.f) P.Velocity += Impulse;
	}

private:
	TArray<FParticle>   Particles;
	TArray<FSolverEdge> Edges;

	TArray<float> Lambda;
	TArray<bool>  BrokenEdges;

	TArray<int32> EdgeColors;
	int32         NumColors = 0;

	FSolverDiagnostics Diagnostics;

	void BuildGraphColoring();
	void ComputeDiagnostics(float DeltaTime, const FSolverConfig& Config);
};
