#include "spatialGridCPU.h"
#include <cmath>
#include <algorithm>

SpatialGridCPU::SpatialGridCPU(float cellSize, int boxSize, int totalBoids)
	: cellSize(cellSize)
{
	gridSize = std::ceil(boxSize / cellSize);
	totalCells = gridSize * gridSize * gridSize;
	cellIndices.resize(totalBoids, -1);
	boidIndices.resize(totalBoids);
	for (int i = 0; i < totalBoids; i++) 
	{
		boidIndices[i] = i;
	}
	cellStart.resize(totalCells, -1);
	cellEnd.resize(totalCells, -1);
}

void SpatialGridCPU::update(float3* positions, int totalBoids)
{
	assignBoidsToCells(positions, totalBoids);
	sortBoids();
	computeCellRanges();
}

void SpatialGridCPU::getGridData(int*& d_cellStart, int*& d_cellEnd, int*& d_boidsIndices)
{
	d_cellStart = cellStart.data();
	d_cellEnd = cellEnd.data();
	d_boidsIndices = boidIndices.data();
}

std::vector<int> SpatialGridCPU::queryNeighborCells(int boidIdx, const float3* positions)
{
	std::vector<int> neighbors;

	float3 pos = positions[boidIdx];

	int cellX = static_cast<int>(pos.x / cellSize);
	int cellY = static_cast<int>(pos.y / cellSize);
	int cellZ = static_cast<int>(pos.z / cellSize);

	for (int dz = -1; dz <= 1; ++dz) 
	{
		for (int dy = -1; dy <= 1; ++dy) 
		{
			for (int dx = -1; dx <= 1; ++dx) 
			{
				int neighborCellX = cellX + dx;
				int neighborCellY = cellY + dy;
				int neighborCellZ = cellZ + dz;

				if (neighborCellX < 0 || neighborCellX >= gridSize ||
					neighborCellY < 0 || neighborCellY >= gridSize ||
					neighborCellZ < 0 || neighborCellZ >= gridSize) 
				{
					continue;
				}

				int neighborCellIdx = neighborCellX
					+ neighborCellY * gridSize
					+ neighborCellZ * gridSize * gridSize;

				int start = cellStart[neighborCellIdx];
				int end = cellEnd[neighborCellIdx];

				if (start == -1 || end == -1) continue;

				for (int i = start; i <= end; ++i) 
				{
					neighbors.push_back(boidIndices[i]);
				}
			}
		}
	}
	return neighbors;
}

void SpatialGridCPU::assignBoidsToCells(float3* positions, int totalBoids)
{
	for (int i = 0; i < totalBoids; i++)
	{
		float3 pos = positions[i];
		int cellX = static_cast<int>(pos.x / cellSize);
		int cellY = static_cast<int>(pos.y / cellSize);
		int cellZ = static_cast<int>(pos.z / cellSize);

		cellX = std::max(0, std::min(cellX, gridSize - 1));
		cellY = std::max(0, std::min(cellY, gridSize - 1));
		cellZ = std::max(0, std::min(cellZ, gridSize - 1));

		cellIndices[i] = cellX + cellY * gridSize + cellZ * gridSize * gridSize;
	}
}

void SpatialGridCPU::sortBoids()
{
	std::vector<std::pair<int, int>> boidCellPairs;
	for (int i = 0; i < cellIndices.size(); i++)
	{
		boidCellPairs.emplace_back(cellIndices[i], boidIndices[i]);
	}

	std::sort(boidCellPairs.begin(), boidCellPairs.end());

	for (int i = 0; i < boidCellPairs.size(); i++)
	{
		cellIndices[i] = boidCellPairs[i].first;
		boidIndices[i] = boidCellPairs[i].second;
	}
}

void SpatialGridCPU::computeCellRanges()
{
	std::fill(cellStart.begin(), cellStart.end(), -1);
	std::fill(cellEnd.begin(), cellEnd.end(), -1);

	for (int i = 0; i < cellIndices.size(); ++i) {
		int cellIdx = cellIndices[i];

		if (cellStart[cellIdx] == -1) {
			cellStart[cellIdx] = i;
		}

		cellEnd[cellIdx] = i;
	}
}

