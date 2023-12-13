#include "Simulation.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cmath>

Simulation::Simulation(Boids& boidsStruct, SpatialGrid& spatialGrid)
	: separationFactor(0.5f),
	alignmentFactor(0.0005f),
	cohesionFactor(0.0005f),
	maxSpeed(70.0f),
	minSpeed(35.0f),
	grid(spatialGrid),
	boids(boidsStruct)
{ }

void Simulation::runSimulationFrame(float deltaTime)
{
	grid.updateBoidCellMap(boids, boids.boidsNumber);
	// Update the velocities based on the simulation rules
	for (int i = 0; i < boids.boidsNumber; i++)
	{
		//std::cout << "Hash of " << i << " is " << grid.hashBoid(boids, i) << std::endl;
		std::vector<int> nhoodBoids = grid.getBoidsFromRegion(grid.hashBoid(boids, i));
		std::vector<int> boidsInRadius;
		std::copy_if(nhoodBoids.begin(), nhoodBoids.end(), std::back_inserter(boidsInRadius), [i, this](int boid) {return isWithinRadius(i, boid, 25); });
		separation(boidsInRadius, i);
		alignment(boidsInRadius, i);
		cohesion(boidsInRadius, i);
	}
}

void Simulation::separation(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVecSum = 0.0f, yVecSum = 0.0f;
	
	// Calculate the change in velocity based on the interaction with
	// other boids in the neighborhood

	for (auto boid : nhoodBoids)
	{
		xVecSum += (boids.positionX[ownId] - boids.positionX[boid]);
		yVecSum += (boids.positionY[ownId] - boids.positionY[boid]);
	}
	
	updateVelocity(xVecSum * separationFactor, yVecSum * separationFactor, ownId);

	/*boids.velocityX[ownId] += (xVecSum * separationFactor);
	boids.velocityY[ownId] += (yVecSum * separationFactor);*/
}

void Simulation::alignment(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVecSum = 0, yVecSum = 0;
	for (auto boid : nhoodBoids)
	{
		xVecSum += (boids.positionX[ownId] - boids.positionX[boid]);
		yVecSum += (boids.positionY[ownId] - boids.positionY[boid]);
	}
	xVecSum *= alignmentFactor;
	yVecSum *= alignmentFactor;

	updateVelocity(boids.velocityX[ownId] - xVecSum,
		boids.velocityY[ownId] - yVecSum, ownId);

	/*boids.velocityX[ownId] += (boids.velocityX[ownId] - xVecSum);
	boids.velocityY[ownId] += (boids.velocityY[ownId] - yVecSum);*/
}

void Simulation::cohesion(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xPosAvg = 0.0f, yPosAvg = 0.0f;
	for (auto boid : nhoodBoids)
	{
		xPosAvg += boids.positionX[boid];
		yPosAvg += boids.positionY[boid];
	}
	xPosAvg /= nhoodBoids.size();
	yPosAvg /= nhoodBoids.size();

	updateVelocity((xPosAvg - boids.positionX[ownId]) * cohesionFactor,
		(yPosAvg - boids.positionY[ownId]) * cohesionFactor, ownId);

	/*boids.velocityX[ownId] += (xPosAvg - boids.positionX[ownId]) * cohesionFactor;
	boids.velocityY[ownId] += (yPosAvg - boids.positionY[ownId]) * cohesionFactor;*/
}

void Simulation::updateVelocity(float xVelocity, float yVelocity, int boidId)
{
	float xNewVelocity = boids.velocityX[boidId] + xVelocity;
	float yNewVelocity = boids.velocityY[boidId] + yVelocity;

	float newSpeed = sqrt(xNewVelocity * xNewVelocity + yNewVelocity * yNewVelocity);

	if (newSpeed > maxSpeed)
	{
		xNewVelocity *= (maxSpeed / newSpeed);
		yNewVelocity *= (maxSpeed / newSpeed);
	}
	if (newSpeed < minSpeed)
	{
		xNewVelocity *= (minSpeed / newSpeed);
		yNewVelocity *= (minSpeed / newSpeed);
	}

	boids.velocityX[boidId] = xNewVelocity;
	boids.velocityY[boidId] = yNewVelocity;
}

bool Simulation::isWithinRadius(int centerBoid, int otherBoid, int radius)
{
	float xDistance = std::abs(boids.positionX[centerBoid] - boids.positionX[otherBoid]);
	float yDistance = std::abs(boids.positionY[centerBoid] - boids.positionY[otherBoid]);
	float distance = std::sqrt(xDistance * xDistance + yDistance * yDistance);

	return distance < radius;

}


float Simulation::vector2Norm(float x1, float x2)
{
	return sqrt(x1 * x1 + x2 * x2);
}