// spatialGridGPU.h
#pragma once
#include <cuda_runtime.h>

class SpatialGridGPU 
{
public:
    SpatialGridGPU(float cellSize, int boxSize, int totalBoids);
    ~SpatialGridGPU();

    void update(float3* positions, int totalBoids);
    void getGridData(int*& d_cellStart, int*& d_cellEnd, int*& d_boidIndices);

private:
    float cellSize;
    int gridSize;
    int totalCells;

    int* d_cellIndices;
    int* d_boidIndices;
    int* d_cellStart;
    int* d_cellEnd;
};
