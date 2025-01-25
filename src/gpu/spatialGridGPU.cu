#include "spatialGridGPU.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/sort.h>
#include <thrust/device_ptr.h>
#include <thrust/scan.h>

#define THREAD_COUNT 256

__global__ void assignBoidsToCellsKernel(float3* positions, int* cellIndices, int* boidIndices,
    float cellSize, int gridSize, int totalBoids) 
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= totalBoids) return;

    float3 pos = positions[idx];
    int cellX = min(max(static_cast<int>(pos.x / cellSize), 0), gridSize - 1);
    int cellY = min(max(static_cast<int>(pos.y / cellSize), 0), gridSize - 1);
    int cellZ = min(max(static_cast<int>(pos.z / cellSize), 0), gridSize - 1);

    cellIndices[idx] = cellX + cellY * gridSize + cellZ * gridSize * gridSize;
    boidIndices[idx] = idx;
}

__global__ void computeCellRangesKernel(int* cellIndices,
    int* cellStart, int* cellEnd, int totalBoids) 
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= totalBoids) return;

    int cellIdx = cellIndices[idx];
    if (idx == 0 || cellIndices[idx - 1] != cellIdx) 
    {
        cellStart[cellIdx] = idx;
    }
    if (idx == totalBoids - 1 || cellIndices[idx + 1] != cellIdx) 
    {
        cellEnd[cellIdx] = idx;
    }
}

SpatialGridGPU::SpatialGridGPU(float cellSize, int boxSize, int totalBoids)
    : cellSize(cellSize) 
{
    gridSize = ceil(boxSize / cellSize);
    totalCells = gridSize * gridSize * gridSize;

    // Allocate device memory
    cudaMalloc(&d_cellIndices, totalBoids * sizeof(int));
    cudaMalloc(&d_boidIndices, totalBoids * sizeof(int));
    cudaMalloc(&d_cellStart, totalCells * sizeof(int));
    cudaMalloc(&d_cellEnd, totalCells * sizeof(int));
}

SpatialGridGPU::~SpatialGridGPU() 
{
    cudaFree(d_cellIndices);
    cudaFree(d_boidIndices);
    cudaFree(d_cellStart);
    cudaFree(d_cellEnd);
}

void SpatialGridGPU::update(float3* positions, int totalBoids) 
{
    int numBlocks = std::ceil(static_cast<float>(totalBoids) / THREAD_COUNT);

    cudaMemset(d_cellStart, -1, totalCells * sizeof(int));
    cudaMemset(d_cellEnd, -1, totalCells * sizeof(int));

    // Assign boids to cells
    assignBoidsToCellsKernel << <numBlocks, THREAD_COUNT >> > (
        positions, d_cellIndices, d_boidIndices, cellSize, gridSize, totalBoids);

    // Sort by cell indices
    thrust::device_ptr<int> cell_indices_ptr(d_cellIndices);
    thrust::device_ptr<int> boid_indices_ptr(d_boidIndices);
    thrust::sort_by_key(cell_indices_ptr, cell_indices_ptr + totalBoids, boid_indices_ptr);

    // Compute cell ranges
    computeCellRangesKernel << <numBlocks, THREAD_COUNT >> > (
        d_cellIndices, d_cellStart, d_cellEnd, totalBoids);
}

void SpatialGridGPU::getGridData(int*& d_cellStart, int*& d_cellEnd, int*& d_boidIndices) 
{
    d_cellStart = this->d_cellStart;
    d_cellEnd = this->d_cellEnd;
    d_boidIndices = this->d_boidIndices;
}