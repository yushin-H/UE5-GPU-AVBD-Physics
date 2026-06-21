#pragma once

#include "CoreMinimal.h"
#include "SolverTypes.h"

class CUSTOMPHYSICSSOLVER_API IPhysicsSolver
{
public:
	virtual ~IPhysicsSolver() = default;

	virtual void Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges) = 0;

	virtual void Step(float DeltaTime, const FSolverConfig& Config) = 0;

	virtual const TArray<FParticle>& GetParticles() const = 0;

	virtual ESolverType GetType() const = 0;

	virtual FSolverDiagnostics GetDiagnostics() const { return FSolverDiagnostics{}; }

	// Add velocity impulse to all free particles (InvMass > 0)
	virtual void ApplyImpulseToAll(const FVector& Impulse) {}
};
