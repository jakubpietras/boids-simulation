#include "simulationCPU.h"

SimulationCPU& SimulationCPU::getInstance()
{
	static SimulationCPU instance;
	return instance;
}

void SimulationCPU::init(std::shared_ptr<Boids> boidsPtr, int boxSize)
{
	float cellSize = 5.0f;
	boids = boidsPtr;
	grid = std::make_shared<SpatialGridCPU>(cellSize, boxSize, boids->totalBoids);
	grid->update(boidsPtr->h_position.data(), boidsPtr->totalBoids);
}

void SimulationCPU::run(const SimulationState& state, float deltaTime)
{
	if (state.isPlaying)
	{
		for (int boid = 0; boid < boids->totalBoids; boid++)
		{
			handleWallCollisions(state, boid, deltaTime);
		}
		handleFlocking(state, deltaTime);
		boids->updatePositions(deltaTime);
		grid->update(boids->h_position.data(), boids->totalBoids);
	}
}

void SimulationCPU::handleWallCollisions(const SimulationState& state, int boid, float deltaTime)
{
	const float wallMargin = 1.0f;  // distance to start steering away from walls
	auto& pos = boids->h_position[boid];
	auto& vel = boids->h_velocity[boid];
	glm::vec3 steeringForce(0.0f);

	if (pos.x < wallMargin) 
	{
		steeringForce.x += (wallMargin - pos.x) / wallMargin * state.maxSpeed;
		if (pos.x <= 0) 
		{
			pos.x = 0;
			vel.x = std::abs(vel.x);
		}
	}
	else if (pos.x > state.boxSize - wallMargin) 
	{
		steeringForce.x -= (pos.x - (state.boxSize - wallMargin)) / wallMargin * state.maxSpeed;
		if (pos.x >= state.boxSize) 
		{
			pos.x = state.boxSize;
			vel.x = -std::abs(vel.x);
		}
	}

	if (pos.y < wallMargin) 
	{
		steeringForce.y += (wallMargin - pos.y) / wallMargin * state.maxSpeed;
		if (pos.y <= 0) 
		{
			pos.y = 0;
			vel.y = std::abs(vel.y);
		}
	}
	else if (pos.y > state.boxSize - wallMargin) 
	{
		steeringForce.y -= (pos.y - (state.boxSize - wallMargin)) / wallMargin * state.maxSpeed;
		if (pos.y >= state.boxSize) 
		{
			pos.y = state.boxSize;
			vel.y = -std::abs(vel.y);
		}
	}

	if (pos.z < wallMargin) 
	{
		steeringForce.z += (wallMargin - pos.z) / wallMargin * state.maxSpeed;
		if (pos.z <= 0) 
		{
			pos.z = 0;
			vel.z = std::abs(vel.z);
		}
	}
	else if (pos.z > state.boxSize - wallMargin) 
	{
		steeringForce.z -= (pos.z - (state.boxSize - wallMargin)) / wallMargin * state.maxSpeed;
		if (pos.z >= state.boxSize) 
		{
			pos.z = state.boxSize;
			vel.z = -std::abs(vel.z);
		}
	}

	if (glm::length(steeringForce) > 0) 
	{
		float steeringMagnitude = glm::length(steeringForce);
		if (steeringMagnitude > state.maxSpeed) 
		{
			steeringForce = glm::normalize(steeringForce) * state.maxSpeed;
		}

		vel.x += steeringForce.x * deltaTime;
		vel.y += steeringForce.y * deltaTime;
		vel.z += steeringForce.z * deltaTime;
	}
}

void SimulationCPU::handleFlocking(const SimulationState& state, float deltaTime)
{
	std::vector<glm::vec3> steeringForces(boids->totalBoids);

	for (int centerBoid = 0; centerBoid < boids->totalBoids; centerBoid++)
	{
		auto neighborhood = grid->queryNeighborCells(centerBoid, boids->h_position.data());

		glm::vec3 steeringForce = 
			alignment(state, centerBoid, neighborhood) * state.alignmentFactor
			+ cohesion(state, centerBoid, neighborhood) * state.cohesionFactor
			+ separation(state, centerBoid, neighborhood) * state.separationFactor;
		steeringForces[centerBoid] = steeringForce;
	}

	for (int boid = 0; boid < boids->totalBoids; boid++)
	{
		auto& vel = boids->h_velocity[boid];

		vel.x += steeringForces[boid].x * deltaTime;
		vel.y += steeringForces[boid].y * deltaTime;
		vel.z += steeringForces[boid].z * deltaTime;

		float speed = glm::length(glm::vec3(vel.x, vel.y, vel.z));
		if (speed > state.maxSpeed)
		{
			glm::vec3 normalizedVel = glm::normalize(glm::vec3(vel.x, vel.y, vel.z));
			vel.x = normalizedVel.x * state.maxSpeed;
			vel.y = normalizedVel.y * state.maxSpeed;
			vel.z = normalizedVel.z * state.maxSpeed;
		}
	}
}

glm::vec3 SimulationCPU::alignment(const SimulationState& state, int centerBoid, std::vector<int> neighborhood)
{
	glm::vec3 avgVelocity = glm::vec3(0.0f);
	int neighborCount = 0;

	for (auto otherBoid : neighborhood)
	{
		if (otherBoid == centerBoid)
			continue;
		if (boids->calculateDistance(centerBoid, otherBoid) < state.visionRadius)
		{
			const float3& vel = boids->h_velocity[otherBoid];
			avgVelocity += glm::vec3(vel.x, vel.y, vel.z);
			neighborCount++;
		}
	}

	if (neighborCount > 0)
	{
		avgVelocity /= static_cast<float>(neighborCount);

		if (glm::length(avgVelocity) > 0)
			avgVelocity = glm::normalize(avgVelocity) * state.maxSpeed;

		glm::vec3 currentVel = glm::vec3(boids->h_velocity[centerBoid].x,
			boids->h_velocity[centerBoid].y,
			boids->h_velocity[centerBoid].z);
		glm::vec3 steeringForce = avgVelocity - currentVel;

		float maxForce = state.maxSpeed;
		if (glm::length(steeringForce) > maxForce)
			steeringForce = glm::normalize(steeringForce) * maxForce;

		return steeringForce;
	}

	return glm::vec3(0.0f);
}

glm::vec3 SimulationCPU::cohesion(const SimulationState& state, int centerBoid, std::vector<int> neighborhood)
{
	glm::vec3 avgPosition = glm::vec3(0.0f);
	int neighborCount = 0;

	for (auto otherBoid : neighborhood)
	{
		if (otherBoid == centerBoid)
			continue;
		if (boids->calculateDistance(centerBoid, otherBoid) < state.visionRadius)
		{
			const float3& pos = boids->h_position[otherBoid];
			avgPosition += glm::vec3(pos.x, pos.y, pos.z);
			neighborCount++;
		}
	}

	if (neighborCount > 0)
	{
		avgPosition /= static_cast<float>(neighborCount);

		glm::vec3 currentPos = glm::vec3(boids->h_position[centerBoid].x,
			boids->h_position[centerBoid].y,
			boids->h_position[centerBoid].z);
		glm::vec3 desiredDirection = avgPosition - currentPos;

		if (glm::length(desiredDirection) > 0)
			desiredDirection = glm::normalize(desiredDirection) * state.maxSpeed;

		glm::vec3 currentVel = glm::vec3(boids->h_velocity[centerBoid].x,
			boids->h_velocity[centerBoid].y,
			boids->h_velocity[centerBoid].z);
		glm::vec3 steeringForce = desiredDirection - currentVel;

		float maxForce = state.maxSpeed;
		if (glm::length(steeringForce) > maxForce)
			steeringForce = glm::normalize(steeringForce) * maxForce;
		return steeringForce;
	}

	return glm::vec3(0.0f);
}

glm::vec3 SimulationCPU::separation(const SimulationState& state, int centerBoid, std::vector<int> neighborhood)
{
	glm::vec3 avgDifferences = glm::vec3(0.0f);
	int neighborCount = 0;

	for (auto otherBoid : neighborhood)
	{
		if (otherBoid == centerBoid)
			continue;
		float distance;
		if ((distance = boids->calculateDistance(centerBoid, otherBoid)) < state.visionRadius)
		{
			const float3& posCenter = boids->h_position[centerBoid];
			const float3& posOther = boids->h_position[otherBoid];
			auto diff = glm::vec3(posCenter.x, posCenter.y, posCenter.z) - glm::vec3(posOther.x, posOther.y, posOther.z);
			diff /= distance;
			avgDifferences += diff;
			neighborCount++;
		}
	}

	if (neighborCount > 0)
	{
		avgDifferences /= static_cast<float>(neighborCount);

		glm::vec3 desiredDirection = avgDifferences;

		if (glm::length(desiredDirection) > 0)
			desiredDirection = glm::normalize(desiredDirection) * state.maxSpeed;

		glm::vec3 currentVel = glm::vec3(boids->h_velocity[centerBoid].x,
			boids->h_velocity[centerBoid].y,
			boids->h_velocity[centerBoid].z);
		glm::vec3 steeringForce = desiredDirection - currentVel;

		float maxForce = state.maxSpeed;
		if (glm::length(steeringForce) > maxForce)
			steeringForce = glm::normalize(steeringForce) * maxForce;
		return steeringForce;
	}

	return glm::vec3(0.0f);
}