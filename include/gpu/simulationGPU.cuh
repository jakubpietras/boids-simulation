#pragma once
#include <memory>
#include "spatialGridGPU.cuh"
#include "boids.h"
#include "simulationState.h"

class SimulationGPU 
{
public:
    static SimulationGPU& getInstance();
    void init(std::shared_ptr<Boids> boidsPtr, int boxSize, unsigned int modelMatVBO);
    void run(const SimulationState& state, float deltaTime);
    void reset();

private:
    SimulationGPU() = default;
    ~SimulationGPU();
    std::shared_ptr<Boids> boids;
    std::shared_ptr<SpatialGridGPU> grid;

    unsigned int vbo;
    cudaGraphicsResource* vboRes;

    float3* d_positions;
    float3* d_velocities;
    float3* d_forces;
};