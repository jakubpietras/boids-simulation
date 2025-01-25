#include "simulationGPU.cuh"
#include <glad/glad.h>
#define GLM_FORCE_CUDA
#include <glm/glm.hpp>
#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>
#include <glm/ext/matrix_transform.hpp>
#include "float4x4.h"
#include <math_constants.h>
#include <cmath>

#define THREAD_COUNT 256

__device__ 
float3 operator+(const float3& a, const float3& b) 
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__device__ 
float3 operator-(const float3& a, const float3& b) 
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ 
float3 operator*(const float3& a, float b) 
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}

__device__ 
float3 operator/(const float3& a, float b) 
{
    float inv = 1.0f / b;
    return a * inv;
}

__device__ 
float length(const float3& v) 
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

__device__ 
float3 normalize(const float3& v) 
{
    float len = length(v);
    if (len > 0) return v / len;
    return v;
}

__device__ 
float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__global__ void computeFlockingForcesKernel(
    float3* positions, 
    float3* velocities, 
    float3* forces,
    int* cellStart, 
    int* cellEnd, 
    int* boidIndices,
    int totalBoids, 
    int gridSize, 
    float cellSize,
    float visionRadius, 
    float visionAngle, 
    float maxSpeed,
    float alignmentFactor, 
    float cohesionFactor, 
    float separationFactor)
{
    int boidIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (boidIdx >= totalBoids) return;

    float3 pos = positions[boidIdx];
    float3 vel = velocities[boidIdx];

    float3 avgVelocity = make_float3(0, 0, 0);
    float3 avgPosition = make_float3(0, 0, 0);
    float3 separation = make_float3(0, 0, 0);
    int neighborCount = 0;

    int cellX = static_cast<int>(pos.x / cellSize);
    int cellY = static_cast<int>(pos.y / cellSize);
    int cellZ = static_cast<int>(pos.z / cellSize);

    const float MIN_DISTANCE = 2.0f;

    // loop over 3x3x3 grid around the boid's cell
    for (int dz = -1; dz <= 1; dz++) 
    {
        for (int dy = -1; dy <= 1; dy++) 
        {
            for (int dx = -1; dx <= 1; dx++) 
            {
                int nx = cellX + dx;
                int ny = cellY + dy;
                int nz = cellZ + dz;

                if (nx < 0 || nx >= gridSize ||
                    ny < 0 || ny >= gridSize ||
                    nz < 0 || nz >= gridSize)
                    continue;

                int cellIdx = nx + ny * gridSize + nz * gridSize * gridSize;
                int start = cellStart[cellIdx]; // where the indices of boids in that cell start
                int end = cellEnd[cellIdx]; // where the indices of boids in that cell end

                if (start == -1 || end == -1) continue;

                for (int otherIdx = start; otherIdx <= end; otherIdx++) 
                {
                    int idx = boidIndices[otherIdx];
                    if (idx != boidIdx) 
                    {
                        float3 otherPos = positions[idx];
                        float3 diff = pos - otherPos;
                        float dist = length(diff);

                        if (dist < visionRadius && dist > 0) 
                        {
                            float3 otherVel = velocities[idx];

                            avgVelocity = avgVelocity + otherVel;
                            avgPosition = avgPosition + otherPos;

                            float separationStrength = (dist < MIN_DISTANCE) ?
                                5.0f * (MIN_DISTANCE - dist) / MIN_DISTANCE : 1.0f; // stronger separation when too close
                            separation = separation + normalize(diff) * separationStrength / dist;
                            neighborCount++;
                        }
                    }
                }
            }
        }
    }

    float3 force = make_float3(0, 0, 0);
    if (neighborCount > 0) 
    {
        avgVelocity = avgVelocity / neighborCount;
        float3 alignmentForce = (normalize(avgVelocity) * maxSpeed - vel) * alignmentFactor;

        avgPosition = avgPosition / neighborCount;
        float3 cohesionForce = (normalize(avgPosition - pos) * maxSpeed - vel) * cohesionFactor;

        separation = separation / neighborCount;
        float3 separationForce = (normalize(separation) * maxSpeed - vel) * separationFactor;

        force = alignmentForce + cohesionForce + separationForce;

        // limit the speed
        float forceMag = length(force);
        if (forceMag > maxSpeed)
            force = normalize(force) * maxSpeed;
    }
    forces[boidIdx] = force;
}

__global__ void updatePositionsKernel(
    float3* positions, 
    float3* velocities, 
    float3* forces,
    int totalBoids, 
    float deltaTime, 
    float maxSpeed,
    int boxSize, 
    float wallMargin)
{
    int boidIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (boidIdx >= totalBoids) return;

    float3 pos = positions[boidIdx];
    float3 vel = velocities[boidIdx];
    float3 force = forces[boidIdx];

    vel = vel + force * deltaTime;
    float3 steeringForce = make_float3(0.0f, 0.0f, 0.0f);

    if (pos.x < wallMargin) 
    {
        steeringForce.x += (wallMargin - pos.x) / wallMargin * maxSpeed;
        if (pos.x <= 0) 
        {
            pos.x = 0;
            vel.x = abs(vel.x);
        }
    }
    else if (pos.x > boxSize - wallMargin) 
    {
        steeringForce.x -= (pos.x - (boxSize - wallMargin)) / wallMargin * maxSpeed;
        if (pos.x >= boxSize) 
        {
            pos.x = boxSize;
            vel.x = -abs(vel.x);
        }
    }

    if (pos.y < wallMargin) 
    {
        steeringForce.y += (wallMargin - pos.y) / wallMargin * maxSpeed;
        if (pos.y <= 0) 
        {
            pos.y = 0;
            vel.y = abs(vel.y);
        }
    }
    else if (pos.y > boxSize - wallMargin) 
    {
        steeringForce.y -= (pos.y - (boxSize - wallMargin)) / wallMargin * maxSpeed;
        if (pos.y >= boxSize) 
        {
            pos.y = boxSize;
            vel.y = -abs(vel.y);
        }
    }

    if (pos.z < wallMargin) 
    {
        steeringForce.z += (wallMargin - pos.z) / wallMargin * maxSpeed;
        if (pos.z <= 0) 
        {
            pos.z = 0;
            vel.z = abs(vel.z);
        }
    }
    else if (pos.z > boxSize - wallMargin) 
    {
        steeringForce.z -= (pos.z - (boxSize - wallMargin)) / wallMargin * maxSpeed;
        if (pos.z >= boxSize) 
        {
            pos.z = boxSize;
            vel.z = -abs(vel.z);
        }
    }

    float steeringMagnitude = length(steeringForce);
    if (steeringMagnitude > maxSpeed) 
    {
        steeringForce = normalize(steeringForce) * maxSpeed;
    }

    vel = vel + steeringForce * deltaTime;

    float speed = length(vel);
    if (speed > maxSpeed) 
    {
        vel = normalize(vel) * maxSpeed;
    }

    pos = pos + vel * deltaTime;
    positions[boidIdx] = pos;
    velocities[boidIdx] = vel;
}

__global__
void updateModelMatricesKernel(float4x4* matrices, float3* position, float3* velocity, int totalBoids)
{
    float scale = 1.0f;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= totalBoids)
        return;

    glm::vec3 vel = { velocity[idx].x, velocity[idx].y, velocity[idx].z };
    glm::vec3 pos = { position[idx].x, position[idx].y, position[idx].z };
    glm::vec3 scaleVec = glm::vec3(scale);
    glm::mat4 model = glm::mat4(1.0f);
    glm::vec3 modelOrientation = { 1.0f, 0.0f, 0.0f }; // orientation of the base model
    glm::vec3 velocityNormalized = glm::normalize(vel);
    glm::vec3 rotationAxis = glm::cross(modelOrientation, velocityNormalized);
    float rotationAngle = glm::acos(glm::dot(modelOrientation, velocityNormalized));

    model = glm::translate(model, pos);
    model = glm::scale(model, scaleVec);
    if (rotationAxis != glm::vec3(0.0f))
        model = glm::rotate(model, rotationAngle, glm::normalize(rotationAxis));

    float4x4 result;
    result.col0 = make_float4(model[0][0], model[0][1], model[0][2], model[0][3]);
    result.col1 = make_float4(model[1][0], model[1][1], model[1][2], model[1][3]);
    result.col2 = make_float4(model[2][0], model[2][1], model[2][2], model[2][3]);
    result.col3 = make_float4(model[3][0], model[3][1], model[3][2], model[3][3]);

    matrices[idx] = result;
}

SimulationGPU& SimulationGPU::getInstance() 
{
    static SimulationGPU instance;
    return instance;
}

void SimulationGPU::init(std::shared_ptr<Boids> boidsPtr, int boxSize, unsigned int modelMatVBO)
{
    vbo = modelMatVBO;
    cudaError_t err = cudaGraphicsGLRegisterBuffer(&vboRes, vbo, cudaGraphicsMapFlagsNone);
    if (err != cudaSuccess)
    {
        std::cerr << "Failed to register VBO with CUDA: " << cudaGetErrorString(err) << std::endl;
        throw std::runtime_error("cudaGraphicsGLRegisterBuffer failed");
    }

    err = cudaMalloc(&d_positions, boidsPtr->totalBoids * sizeof(float3));
    if (err != cudaSuccess) 
        throw std::runtime_error("Failed to allocate d_positions");

    err = cudaMalloc(&d_velocities, boidsPtr->totalBoids * sizeof(float3));
    if (err != cudaSuccess) 
        throw std::runtime_error("Failed to allocate d_velocities");

    err = cudaMalloc(&d_forces, boidsPtr->totalBoids * sizeof(float3));
    if (err != cudaSuccess) 
        throw std::runtime_error("Failed to allocate d_forces");

    err = cudaMemcpy(d_positions, boidsPtr->h_position.data(),
        boidsPtr->totalBoids * sizeof(float3), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) 
        throw std::runtime_error("Failed to copy positions");

    err = cudaMemcpy(d_velocities, boidsPtr->h_velocity.data(),
        boidsPtr->totalBoids * sizeof(float3), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) 
        throw std::runtime_error("Failed to copy velocities");

    float cellSize = 5.0f;
    boids = boidsPtr;
    grid = std::make_shared<SpatialGridGPU>(cellSize, boxSize, boids->totalBoids);
}

SimulationGPU::~SimulationGPU() 
{
    cudaError_t err = cudaGraphicsUnregisterResource(vboRes);
    if (err != cudaSuccess)
    {
        std::cerr << "Failed to unregister VBO with CUDA: " << cudaGetErrorString(err) << std::endl;
    }
    cudaFree(d_positions);
    cudaFree(d_velocities);
    cudaFree(d_forces);
}

void SimulationGPU::run(const SimulationState& state, float deltaTime) 
{
    cudaError_t err;
    if (!state.isPlaying) return;

    err = cudaGraphicsMapResources(1, &vboRes, 0);
    if (err != cudaSuccess) 
    {
        std::cerr << "ERROR: cudaGraphicsMapResources failed: " << cudaGetErrorString(err) << std::endl;
        return;
    }

    size_t numBytes = boids->totalBoids * sizeof(float4x4);
    float4x4* modelMatrices;
    err = cudaGraphicsResourceGetMappedPointer((void**)&modelMatrices, &numBytes, vboRes);
    if (err != cudaSuccess) 
    {
        std::cerr << "ERROR: cudaGraphicsResourceGetMappedPointer failed: " << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnmapResources(1, &vboRes, 0);
        return;
    }

    const float wallMargin = 1.0f;
    int numBlocks = std::ceil(static_cast<float>(boids->totalBoids) / THREAD_COUNT);

    grid->update(d_positions, boids->totalBoids);
    int* d_cellStart;
    int* d_cellEnd;
    int* d_boidIndices;
    grid->getGridData(d_cellStart, d_cellEnd, d_boidIndices);

    computeFlockingForcesKernel << <numBlocks, THREAD_COUNT >> > (
        d_positions,
        d_velocities,
        d_forces,
        d_cellStart,
        d_cellEnd,
        d_boidIndices,
        boids->totalBoids,
        static_cast<int>(ceil(state.boxSize / 5.0f)),
        5.0f,
        state.visionRadius,
        state.visionAngle,
        state.maxSpeed,
        state.alignmentFactor,
        state.cohesionFactor,
        state.separationFactor
        );

    updatePositionsKernel << <numBlocks, THREAD_COUNT >> > (
        d_positions,
        d_velocities,
        d_forces,
        boids->totalBoids,
        deltaTime,
        state.maxSpeed,
        state.boxSize,
        wallMargin
        );

    updateModelMatricesKernel << <numBlocks, THREAD_COUNT >> > (
        modelMatrices,
        d_positions,
        d_velocities,
        boids->totalBoids
        );

    err = cudaGetLastError();
    if (err != cudaSuccess) 
    {
        std::cerr << "Kernel launch error: " << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnmapResources(1, &vboRes, 0);
        return;
    }

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) 
    {
        std::cerr << "Kernel execution failed: " << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnmapResources(1, &vboRes, 0);
        return;
    }

    err = cudaGraphicsUnmapResources(1, &vboRes, 0);
    if (err != cudaSuccess) 
    {
        std::cerr << "ERROR: cudaGraphicsUnmapResources failed: " << cudaGetErrorString(err) << std::endl;
        return;
    }
}

void SimulationGPU::reset()
{
    cudaError_t err;

    // unregister the VBO resource
    if (vboRes)
    {
        err = cudaGraphicsUnregisterResource(vboRes);
        if (err != cudaSuccess)
        {
            std::cerr << "ERROR: cudaGraphicsUnregisterResource failed during reset: " << cudaGetErrorString(err) << std::endl;
        }
        vboRes = nullptr;
    }

    // re-register the VBO resource with CUDA
    err = cudaGraphicsGLRegisterBuffer(&vboRes, vbo, cudaGraphicsMapFlagsNone);
    if (err != cudaSuccess)
    {
        std::cerr << "ERROR: cudaGraphicsGLRegisterBuffer failed during reset: " << cudaGetErrorString(err) << std::endl;
        throw std::runtime_error("cudaGraphicsGLRegisterBuffer failed");
    }
}
