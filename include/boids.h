#ifndef PARTICLES_H
#define PARTICLES_H
#include <iostream>
#include <random>
#include "glm/glm.hpp"
#include <cuda_runtime.h>
#include <vector_types.h>
#include "simulationState.h"

class Boids
{
public:
	// cpu memory
	std::vector<float3> h_position, h_velocity;
	size_t totalBoids;

	// gpu memory
	float3* d_position;
	float3* d_velocity;

	Boids(size_t totalBoids, float minSpeed, float maxSpeed, const int boxSize);
	~Boids();
	void updatePositions(float deltaTime);
	float calculateDistance(int centerBoid, int otherBoid);
private:
	void allocateDeviceMemory();
    void generateRandomPositions(const int boxSize);
    void generateRandomVelocities(float minSpeed, float maxSpeed);
};

#endif