#pragma once
#include <cmath>
#include <iostream>
#include <algorithm>
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
	bool isWithinRadius(int centerBoid, int otherBoid);
	bool isWithinFieldOfView(int centerBoid, int otherBoid);
};