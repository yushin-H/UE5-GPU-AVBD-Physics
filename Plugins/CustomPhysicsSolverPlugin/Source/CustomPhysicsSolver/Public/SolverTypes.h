#pragma once

#include "CoreMinimal.h"
#include "SolverTypes.generated.h"

UENUM(BlueprintType)
enum class ESolverType : uint8
{
	PBD      UMETA(DisplayName = "PBD (CPU)"),
	AVBD     UMETA(DisplayName = "AVBD (CPU)"),
	AVBD_GPU UMETA(DisplayName = "AVBD (GPU)"),
};

USTRUCT(BlueprintType)
struct CUSTOMPHYSICSSOLVER_API FParticle
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Position = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector PrevPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Velocity = FVector::ZeroVector;

	// 0 = static/kinematic
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float InvMass = 1.0f;
};

USTRUCT(BlueprintType)
struct CUSTOMPHYSICSSOLVER_API FSolverEdge
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 A = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 B = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float RestLength = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Stiffness = 1.0f;

	// Edge breaks when stretch exceeds this value (cm). 0 = never breaks.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float BreakThreshold = 0.f;
};

USTRUCT(BlueprintType)
struct CUSTOMPHYSICSSOLVER_API FSolverConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Iterations = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Damping = 0.99f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Gravity = FVector(0.f, 0.f, -980.f);

	// Augmented Lagrangian penalty parameter ρ
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float AugLagrangianRho = 100.f;

	// Ground plane Z (world space, cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float FloorZ = 0.f;

	// Velocity restitution on floor contact (0=no bounce, 1=perfect bounce)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(ClampMin=0.0, ClampMax=1.0))
	float Restitution = 0.1f;

	// Tangential friction on floor contact (0=frictionless, 1=full stop)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(ClampMin=0.0, ClampMax=1.0))
	float FloorFriction = 0.6f;
};

// Per-step numerical diagnostics
USTRUCT(BlueprintType)
struct CUSTOMPHYSICSSOLVER_API FSolverDiagnostics
{
	GENERATED_BODY()

	// Σ 0.5 * m_i * |v_i|²
	UPROPERTY(BlueprintReadOnly)
	float KineticEnergy = 0.f;

	// Σ m_i * |g| * z_i  (positive up)
	UPROPERTY(BlueprintReadOnly)
	float PotentialEnergy = 0.f;

	// max |C_c| over all constraints
	UPROPERTY(BlueprintReadOnly)
	float MaxConstraintResidual = 0.f;

	// Σ |C_c| / N
	UPROPERTY(BlueprintReadOnly)
	float AvgConstraintResidual = 0.f;

	// Wall-clock time for one Step() call, milliseconds
	UPROPERTY(BlueprintReadOnly)
	float StepTimeMs = 0.f;
};
