#ifndef SPATIAL_GRID_CPU_H
#define SPATIAL_GRID_CPU_H
#include <vector>
#include <cuda_runtime.h>

class SpatialGridCPU
{
public:
	float cellSize;
	int gridSize, totalCells;

	std::vector<int> cellIndices;
	std::vector<int> boidIndices;
	std::vector<int> cellStart;
	std::vector<int> cellEnd;

	SpatialGridCPU(float cellSize, int boxSize, int totalBoids);
	void update(float3* positions, int totalBoids);
	void getGridData(int*& d_cellStart, int*& d_cellEnd, int*& d_boidsIndices);
	std::vector<int> queryNeighborCells(int boidIdx, const float3* positions);
private:
	void assignBoidsToCells(float3* positions, int totalBoids);
	void sortBoids();
	void computeCellRanges();
};

#endif