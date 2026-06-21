#include "AVBDSolverGPU.h"
#include "AVBDSolver.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "ShaderParameterUtils.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "RHIGPUReadback.h"        // FRHIGPUBufferReadback

// ---------------------------------------------------------------------------
// Compute shader declaration
// ---------------------------------------------------------------------------

class FAVBDPrimalSolveCS : public FGlobalShader
{
	DECLARE_GLOBAL_SHADER(FAVBDPrimalSolveCS);
	SHADER_USE_PARAMETER_STRUCT(FAVBDPrimalSolveCS, FGlobalShader);

	static constexpr uint32 ThreadGroupSize = 64;

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER(int32,  ColorOffset)
		SHADER_PARAMETER(int32,  ColorCount)
		SHADER_PARAMETER(float,  Rho)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, RWParticles)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>,   Edges)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>,  RWLambda)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>,      ColorEdgeIndices)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& P)
	{
		return IsFeatureLevelSupported(P.Platform, ERHIFeatureLevel::SM5);
	}
	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& P,
	                                          FShaderCompilerEnvironment& OutEnv)
	{
		FGlobalShader::ModifyCompilationEnvironment(P, OutEnv);
		OutEnv.SetDefine(TEXT("THREADGROUP_SIZE"), ThreadGroupSize);
	}
};

IMPLEMENT_GLOBAL_SHADER(FAVBDPrimalSolveCS,
	"/Plugin/CustomPhysicsSolver/Private/AVBD_PrimalSolve.usf",
	"MainCS", SF_Compute);

// ---------------------------------------------------------------------------
// FAVBDSolverGPU
// ---------------------------------------------------------------------------

FAVBDSolverGPU::~FAVBDSolverGPU()
{
	// Release pooled buffers (they are ref-counted; releasing here is enough)
	ParticlePooled.SafeRelease();
	EdgePooled.SafeRelease();
	LambdaPooled.SafeRelease();
	ColorIndexPooled.SafeRelease();
}

void FAVBDSolverGPU::Initialize(const TArray<FParticle>& InParticles, const TArray<FSolverEdge>& InEdges)
{
	Particles = InParticles;
	Edges     = InEdges;
	Lambda.Init(0.f, Edges.Num());
	BrokenEdges.Init(false, Edges.Num());

	BuildGraphColoring();
	BuildColorIndexArray();
	InitGPUBuffers();

	UE_LOG(LogTemp, Log, TEXT("[AVBD_GPU] Initialized: %d particles, %d edges, %d colors"),
		Particles.Num(), Edges.Num(), NumColors);
}

// ---------------------------------------------------------------------------
// Graph coloring (same greedy algorithm as CPU solver)
// ---------------------------------------------------------------------------

void FAVBDSolverGPU::BuildGraphColoring()
{
	const int32 NumEdges    = Edges.Num();
	const int32 NumParticles = Particles.Num();

	EdgeColors.Init(-1, NumEdges);
	NumColors = 0;

	TArray<TArray<int32>> Adj;
	Adj.SetNum(NumParticles);
	for (int32 ei = 0; ei < NumEdges; ++ei)
	{
		Adj[Edges[ei].A].Add(ei);
		Adj[Edges[ei].B].Add(ei);
	}

	for (int32 ei = 0; ei < NumEdges; ++ei)
	{
		TSet<int32> Forbidden;
		for (int32 nb : Adj[Edges[ei].A])
			if (nb != ei && EdgeColors[nb] >= 0) Forbidden.Add(EdgeColors[nb]);
		for (int32 nb : Adj[Edges[ei].B])
			if (nb != ei && EdgeColors[nb] >= 0) Forbidden.Add(EdgeColors[nb]);

		int32 Color = 0;
		while (Forbidden.Contains(Color)) ++Color;

		EdgeColors[ei] = Color;
		NumColors = FMath::Max(NumColors, Color + 1);
	}
}

void FAVBDSolverGPU::BuildColorIndexArray()
{
	ColorOffsets.SetNum(NumColors);
	ColorCounts.SetNum(NumColors);
	ColorEdgeIndices.Reset();

	TArray<TArray<int32>> ByColor;
	ByColor.SetNum(NumColors);
	for (int32 ei = 0; ei < Edges.Num(); ++ei)
		ByColor[EdgeColors[ei]].Add(ei);

	for (int32 c = 0; c < NumColors; ++c)
	{
		ColorOffsets[c] = ColorEdgeIndices.Num();
		ColorCounts[c]  = ByColor[c].Num();
		ColorEdgeIndices.Append(ByColor[c]);
	}
}

// ---------------------------------------------------------------------------
// GPU buffer creation
// ---------------------------------------------------------------------------

void FAVBDSolverGPU::InitGPUBuffers()
{
	// Build CPU-side GPU structs
	TArray<FGPUParticle> GpuParticles;
	GpuParticles.SetNum(Particles.Num());
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		GpuParticles[i].Position     = FVector3f(Particles[i].Position);
		GpuParticles[i].InvMass      = Particles[i].InvMass;
		GpuParticles[i].PrevPosition = FVector3f(Particles[i].PrevPosition);
		GpuParticles[i].Velocity     = FVector3f(Particles[i].Velocity);
	}

	TArray<FGPUEdge> GpuEdges;
	GpuEdges.SetNum(Edges.Num());
	for (int32 i = 0; i < Edges.Num(); ++i)
	{
		GpuEdges[i].A          = Edges[i].A;
		GpuEdges[i].B          = Edges[i].B;
		GpuEdges[i].RestLength = Edges[i].RestLength;
	}

	TArray<float> GpuLambda = Lambda;

	// Capture by value for render thread
	ENQUEUE_RENDER_COMMAND(AVBDGPUInit)(
	[this,
	 GpuParticles  = MoveTemp(GpuParticles),
	 GpuEdges      = MoveTemp(GpuEdges),
	 GpuLambda     = MoveTemp(GpuLambda),
	 ColorIdxCopy  = ColorEdgeIndices](FRHICommandListImmediate& RHICmdList)
	{
		FRDGBuilder GraphBuilder(RHICmdList);

		auto UploadBuffer = [&](auto& Data, uint32 Stride, const TCHAR* Name,
		                        TRefCountPtr<FRDGPooledBuffer>& OutPooled)
		{
			const uint32 NumElements = (uint32)Data.Num();
			FRDGBufferRef Buf = GraphBuilder.CreateBuffer(
				FRDGBufferDesc::CreateStructuredDesc(Stride, FMath::Max(NumElements, 1u)),
				Name);
			GraphBuilder.QueueBufferUpload(Buf, Data.GetData(),
				Stride * NumElements, ERDGInitialDataFlags::NoCopy);
			GraphBuilder.QueueBufferExtraction(Buf, &OutPooled, ERHIAccess::UAVCompute);
		};

		UploadBuffer(GpuParticles, sizeof(FGPUParticle), TEXT("AVBD_Particles"),   ParticlePooled);
		UploadBuffer(GpuEdges,     sizeof(FGPUEdge),     TEXT("AVBD_Edges"),       EdgePooled);
		UploadBuffer(GpuLambda,    sizeof(float),         TEXT("AVBD_Lambda"),      LambdaPooled);
		UploadBuffer(ColorIdxCopy, sizeof(int32),         TEXT("AVBD_ColorIndex"),  ColorIndexPooled);

		GraphBuilder.Execute();
		bGPUBuffersReady = true;
	});

	FlushRenderingCommands();
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

void FAVBDSolverGPU::Step(float DeltaTime, const FSolverConfig& Config)
{
	if (!bGPUBuffersReady) return;

	const double T0 = FPlatformTime::Seconds();
	const float h  = DeltaTime;
	const float h2 = h * h;

	// --- 1. Save x_prev, apply predict on CPU ---
	TArray<FVector> XPrev;
	XPrev.SetNum(Particles.Num());
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		XPrev[i] = Particles[i].Position;
		if (Particles[i].InvMass > 0.f)
			Particles[i].Position += Particles[i].Velocity * h + Config.Gravity * h2;
	}

	// --- 2+3+4. Upload + Primal Solve + Readback (single render command) ---
	//  Batching all GPU work into one RDG graph eliminates two intermediate
	//  FlushRenderingCommands stalls, reducing CPU-GPU round trips from 3 to 1.
	const int32 N = Particles.Num();

	TArray<FGPUParticle> Upload;
	Upload.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Upload[i].Position     = FVector3f(Particles[i].Position);
		Upload[i].InvMass      = Particles[i].InvMass;
		Upload[i].PrevPosition = FVector3f(Particles[i].PrevPosition);
		Upload[i].Velocity     = FVector3f(Particles[i].Velocity);
	}

	TArray<float> LambdaCopy  = Lambda;
	TArray<int32> OffsetsCopy = ColorOffsets;
	TArray<int32> CountsCopy  = ColorCounts;

	TArray<FGPUParticle> Readback;
	Readback.SetNum(N);

	const int32 Iterations = Config.Iterations;
	const float Rho        = Config.AugLagrangianRho;

	ENQUEUE_RENDER_COMMAND(AVBDFullStep)(
	[this, N, Iterations, Rho,
	 Upload      = MoveTemp(Upload),
	 LambdaCopy  = MoveTemp(LambdaCopy),
	 OffsetsCopy = MoveTemp(OffsetsCopy),
	 CountsCopy  = MoveTemp(CountsCopy),
	 &Readback](FRHICommandListImmediate& RHICmdList)
	{
		FRDGBuilder GraphBuilder(RHICmdList);

		FRDGBufferRef ParticleBuf = GraphBuilder.RegisterExternalBuffer(ParticlePooled);
		FRDGBufferRef EdgeBuf     = GraphBuilder.RegisterExternalBuffer(EdgePooled);
		FRDGBufferRef LambdaBuf   = GraphBuilder.RegisterExternalBuffer(LambdaPooled);
		FRDGBufferRef ColorIdxBuf = GraphBuilder.RegisterExternalBuffer(ColorIndexPooled);

		GraphBuilder.QueueBufferUpload(ParticleBuf,
			Upload.GetData(), sizeof(FGPUParticle) * Upload.Num(),
			ERDGInitialDataFlags::NoCopy);
		GraphBuilder.QueueBufferUpload(LambdaBuf,
			LambdaCopy.GetData(), sizeof(float) * LambdaCopy.Num(),
			ERDGInitialDataFlags::NoCopy);

		FRDGBufferUAVRef ParticleUAV = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(ParticleBuf));
		FRDGBufferSRVRef EdgeSRV     = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(EdgeBuf));
		FRDGBufferUAVRef LambdaUAV   = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(LambdaBuf));
		FRDGBufferSRVRef ColorIdxSRV = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(ColorIdxBuf));

		TShaderMapRef<FAVBDPrimalSolveCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
		const int32 NC = CountsCopy.Num();

		for (int32 Iter = 0; Iter < Iterations; ++Iter)
		{
			for (int32 Color = 0; Color < NC; ++Color)
			{
				const int32 Count = CountsCopy[Color];
				if (Count <= 0) continue;

				auto* Params = GraphBuilder.AllocParameters<FAVBDPrimalSolveCS::FParameters>();
				Params->ColorOffset      = OffsetsCopy[Color];
				Params->ColorCount       = Count;
				Params->Rho              = Rho;
				Params->RWParticles      = ParticleUAV;
				Params->Edges            = EdgeSRV;
				Params->RWLambda         = LambdaUAV;
				Params->ColorEdgeIndices = ColorIdxSRV;

				FComputeShaderUtils::AddPass(GraphBuilder,
					RDG_EVENT_NAME("AVBD_Primal[iter=%d color=%d]", Iter, Color),
					Shader, Params,
					FIntVector(FMath::DivideAndRoundUp(Count, 64), 1, 1));
			}
		}

		FRHIGPUBufferReadback GPUReadback(TEXT("AVBD_Readback"));
		AddEnqueueCopyPass(GraphBuilder, &GPUReadback, ParticleBuf, sizeof(FGPUParticle) * N);

		GraphBuilder.Execute();

		RHICmdList.ImmediateFlush(EImmediateFlushType::FlushRHIThread);
		while (!GPUReadback.IsReady()) { FPlatformProcess::Sleep(0.f); }

		const FGPUParticle* Src = static_cast<const FGPUParticle*>(
			GPUReadback.Lock(sizeof(FGPUParticle) * N));
		FMemory::Memcpy(Readback.GetData(), Src, sizeof(FGPUParticle) * N);
		GPUReadback.Unlock();
	});
	FlushRenderingCommands();

	for (int32 i = 0; i < N; ++i)
		Particles[i].Position = FVector(Readback[i].Position);

	// --- 4b. Check constraint breaking (uses CPU positions from readback) ---
	CheckAndBreakEdges();

	// --- 4c. Dual update (CPU): λ_c += ρ · C_c(x*)
	//  Runs on CPU using readback positions. Updated Lambda is uploaded to GPU
	//  at the start of the next frame (step 2), completing the Primal-Dual split.
	for (int32 ei = 0; ei < Edges.Num(); ++ei)
	{
		if (BrokenEdges[ei]) continue;
		const FSolverEdge& E = Edges[ei];
		const float C = (Particles[E.B].Position - Particles[E.A].Position).Size() - E.RestLength;
		Lambda[ei] += Config.AugLagrangianRho * C;
	}

	// --- 5. Floor collision (position projection, CPU side after readback) ---
	for (FParticle& P : Particles)
	{
		if (P.InvMass <= 0.f || P.Position.Z >= Config.FloorZ) continue;
		P.Position.Z = Config.FloorZ;
	}

	// --- 6. Velocity update on CPU ---
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		FParticle& P = Particles[i];
		if (P.InvMass <= 0.f) continue;
		P.Velocity     = (P.Position - XPrev[i]) / h;
		P.Velocity    *= Config.Damping;

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

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void FAVBDSolverGPU::ComputeDiagnostics(float h, const FSolverConfig& Config)
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

	float  Sum   = 0.f;
	int32  Count = 0;
	for (const FSolverEdge& E : Edges)
	{
		const float C = (Particles[E.B].Position - Particles[E.A].Position).Size() - E.RestLength;
		if (!FMath::IsFinite(C)) continue;
		const float AbsC = FMath::Abs(C);
		Diagnostics.MaxConstraintResidual = FMath::Max(Diagnostics.MaxConstraintResidual, AbsC);
		Sum += AbsC;
		++Count;
	}
	if (Count > 0) Diagnostics.AvgConstraintResidual = Sum / Count;
}

// ---------------------------------------------------------------------------
// Constraint breaking
// ---------------------------------------------------------------------------

void FAVBDSolverGPU::CheckAndBreakEdges()
{
	bool bAnyBroke = false;
	for (int32 ei = 0; ei < Edges.Num(); ++ei)
	{
		if (BrokenEdges[ei]) continue;
		const FSolverEdge& E = Edges[ei];
		if (E.BreakThreshold <= 0.f) continue;

		const float C = (Particles[E.B].Position - Particles[E.A].Position).Size() - E.RestLength;
		if (C > E.BreakThreshold)
		{
			BrokenEdges[ei] = true;
			bAnyBroke = true;
		}
	}

	if (bAnyBroke)
		RebuildColorIndexGPU();
}

void FAVBDSolverGPU::RebuildColorIndexGPU()
{
	// Rebuild ColorEdgeIndices filtering out broken edges
	TArray<TArray<int32>> ByColor;
	ByColor.SetNum(NumColors);
	for (int32 ei = 0; ei < Edges.Num(); ++ei)
	{
		if (!BrokenEdges[ei])
			ByColor[EdgeColors[ei]].Add(ei);
	}

	ColorOffsets.SetNum(NumColors);
	ColorCounts.SetNum(NumColors);
	ColorEdgeIndices.Reset();
	for (int32 c = 0; c < NumColors; ++c)
	{
		ColorOffsets[c] = ColorEdgeIndices.Num();
		ColorCounts[c]  = ByColor[c].Num();
		ColorEdgeIndices.Append(ByColor[c]);
	}

	if (ColorEdgeIndices.Num() == 0) return;

	TArray<int32> IdxCopy = ColorEdgeIndices;
	ENQUEUE_RENDER_COMMAND(AVBDRebuildColorIdx)(
	[this, IdxCopy = MoveTemp(IdxCopy)](FRHICommandListImmediate& RHICmdList)
	{
		if (!ColorIndexPooled.IsValid()) return;
		FRDGBuilder GraphBuilder(RHICmdList);
		FRDGBufferRef Buf = GraphBuilder.RegisterExternalBuffer(ColorIndexPooled);
		GraphBuilder.QueueBufferUpload(Buf, IdxCopy.GetData(),
			sizeof(int32) * IdxCopy.Num(), ERDGInitialDataFlags::NoCopy);
		GraphBuilder.Execute();
	});
	FlushRenderingCommands();
}
