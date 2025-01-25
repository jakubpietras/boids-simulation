#ifndef SIMULATIONCPU_H
#define SIMULATIONCPU_H
#include "boids.h"
#include "simulationState.h"
#include <cuda_runtime.h>
#include <memory>
#include "spatialGridCPU.h"

class SimulationCPU
{
public:
	static SimulationCPU& getInstance();
	void init(std::shared_ptr<Boids> boidsPtr, int boxSize);
	void run(const SimulationState& state, float deltaTime);
private:
	std::shared_ptr<Boids> boids;
	std::shared_ptr<SpatialGridCPU> grid;
	
	SimulationCPU() = default;
	void handleWallCollisions(const SimulationState& state, int centerBoid, float deltaTime);
	void handleFlocking(const SimulationState& state, float deltaTime);
	glm::vec3 alignment(const SimulationState& state, int centerBoid, std::vector<int> neighborhood);
	glm::vec3 cohesion(const SimulationState& state, int centerBoid, std::vector<int> neighborhood);
	glm::vec3 separation(const SimulationState& state, int centerBoid, std::vector<int> neighborhood);
};

#endif