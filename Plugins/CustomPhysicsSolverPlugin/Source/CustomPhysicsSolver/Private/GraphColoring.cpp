#include "AVBDSolver.h"

void FAVBDSolver::BuildGraphColoring()
{
	const int32 NumEdges    = Edges.Num();
	const int32 NumParticles = Particles.Num();

	EdgeColors.Init(-1, NumEdges);
	NumColors = 0;

	// vertex → incident edge indices
	TArray<TArray<int32>> Adj;
	Adj.SetNum(NumParticles);
	for (int32 ei = 0; ei < NumEdges; ++ei)
	{
		Adj[Edges[ei].A].Add(ei);
		Adj[Edges[ei].B].Add(ei);
	}

	// Greedy edge coloring: assign smallest color not used by any adjacent edge
	for (int32 ei = 0; ei < NumEdges; ++ei)
	{
		TSet<int32> Forbidden;

		// All edges sharing vertex A or B that are already colored
		for (int32 nb : Adj[Edges[ei].A])
		{
			if (nb != ei && EdgeColors[nb] >= 0) Forbidden.Add(EdgeColors[nb]);
		}
		for (int32 nb : Adj[Edges[ei].B])
		{
			if (nb != ei && EdgeColors[nb] >= 0) Forbidden.Add(EdgeColors[nb]);
		}

		int32 Color = 0;
		while (Forbidden.Contains(Color)) ++Color;

		EdgeColors[ei] = Color;
		NumColors = FMath::Max(NumColors, Color + 1);
	}

	UE_LOG(LogTemp, Log, TEXT("GraphColoring: %d edges → %d colors"), NumEdges, NumColors);
}

bool FAVBDSolver::ValidateColoring() const
{
	const int32 NumParticles = Particles.Num();

	// vertex → last seen color for each pass; detect same-color adjacent edges
	TArray<int32> LastColor;
	LastColor.Init(-1, NumParticles);

	TArray<int32> LastEdge;
	LastEdge.Init(-1, NumParticles);

	for (int32 ei = 0; ei < Edges.Num(); ++ei)
	{
		const int32 C  = EdgeColors[ei];
		const int32 vA = Edges[ei].A;
		const int32 vB = Edges[ei].B;

		// Check vertex A
		if (LastColor[vA] == C && LastEdge[vA] != ei)
		{
			UE_LOG(LogTemp, Error,
				TEXT("GraphColoring INVALID: edge %d and edge %d share vertex %d and both have color %d"),
				LastEdge[vA], ei, vA, C);
			return false;
		}
		// Check vertex B
		if (LastColor[vB] == C && LastEdge[vB] != ei)
		{
			UE_LOG(LogTemp, Error,
				TEXT("GraphColoring INVALID: edge %d and edge %d share vertex %d and both have color %d"),
				LastEdge[vB], ei, vB, C);
			return false;
		}

		LastColor[vA] = C;  LastEdge[vA] = ei;
		LastColor[vB] = C;  LastEdge[vB] = ei;
	}

	UE_LOG(LogTemp, Log, TEXT("GraphColoring VALID: %d edges, %d colors"), Edges.Num(), NumColors);
	return true;
}
