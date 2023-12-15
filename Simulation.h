#pragma once
#include "Boids.h"
#include "SpatialGrid.h"

class Simulation
{
public:
	float separationFactor, alignmentFactor, cohesionFactor;
	float maxSpeed, minSpeed;
	float visionRadius;
	Simulation(Boids& boidsStruct, SpatialGrid& spatialGrid, float radius);
	void runSimulationFrame(float deltaTime);

private:
	Boids& boids;
	SpatialGrid& grid;
	void separation(std::vector<int> nhoodBoids, int ownId);
	void alignment(std::vector<int> nhoodBoids, int ownId);
	void cohesion(std::vector<int> nhoodBoids, int ownId);
	void updateVelocity(float xVelocity, float yVelocity, int boidId);
	bool isWithinRadius(int centerBoid, int otherBoid, int radius);
	float vector2Norm(float x1, float x2);
};