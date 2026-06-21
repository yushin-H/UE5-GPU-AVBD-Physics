#pragma once

#include "CoreMinimal.h"
#include "IPhysicsSolver.h"
#include "RHI.h"
#include "RHIResources.h"
#include "RenderGraphResources.h"

// GPU-side particle layout (must match HLSL struct in AVBD_PrimalSolve.usf, 48 bytes)
struct FGPUParticle
{
	FVector3f Position;      // 12
	float     InvMass;       //  4  → 16
	FVector3f PrevPosition;  // 12
	float     Pad0 = 0.f;   //  4  → 32
	FVector3f Velocity;      // 12
	float     Pad1 = 0.f;   //  4  → 48
};
static_assert(sizeof(FGPUParticle) == 48, "FGPUParticle size mismatch");

// GPU-side edge layout (must match HLSL struct, 16 bytes)
struct FGPUEdge
{
	int32 A;
	int32 B;
	float RestLength;
	float Pad = 0.f;
};
static_assert(sizeof(FGPUEdge) == 16, "FGPUEdge size mismatch");

class CUSTOMPHYSICSSOLVER_API FAVBDSolverGPU : public IPhysicsSolver
{
public:
	virtual ~FAVBDSolverGPU();

	virtual void Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges) override;
	virtual void Step(float DeltaTime, const FSolverConfig& Config) override;
	virtual const TArray<FParticle>& GetParticles() const override { return Particles; }
	virtual ESolverType GetType() const override { return ESolverType::AVBD_GPU; }

	virtual FSolverDiagnostics GetDiagnostics() const override { return Diagnostics; }
	virtual void ApplyImpulseToAll(const FVector& Impulse) override
	{
		// Modifies CPU-side velocity; picked up by next frame's GPU upload
		for (FParticle& P : Particles)
			if (P.InvMass > 0.f) P.Velocity += Impulse;
	}

private:
	// CPU-side copies (written back from GPU each frame)
	TArray<FParticle>   Particles;
	TArray<FSolverEdge> Edges;

	// Graph coloring (computed on CPU, uploaded once)
	TArray<int32> EdgeColors;
	int32         NumColors  = 0;
	TArray<int32> ColorOffsets;  // start index per color in ColorEdgeIndices
	TArray<int32> ColorCounts;   // edge count per color
	TArray<int32> ColorEdgeIndices; // edge indices sorted by color

	TArray<float> Lambda;
	TArray<bool>  BrokenEdges;

	// Persistent GPU buffers (allocated on render thread in Initialize)
	TRefCountPtr<FRDGPooledBuffer> ParticlePooled;
	TRefCountPtr<FRDGPooledBuffer> EdgePooled;
	TRefCountPtr<FRDGPooledBuffer> LambdaPooled;
	TRefCountPtr<FRDGPooledBuffer> ColorIndexPooled;

	FSolverDiagnostics Diagnostics;
	bool bGPUBuffersReady = false;

	void BuildGraphColoring();
	void BuildColorIndexArray();
	void InitGPUBuffers();
	void ComputeDiagnostics(float h, const FSolverConfig& Config);
	void CheckAndBreakEdges();
	void RebuildColorIndexGPU();
};
