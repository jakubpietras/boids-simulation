#include "Simulation.h"

Simulation::Simulation(Boids& boidsStruct, SpatialGrid& spatialGrid, float radius, float visionAngle)
	: separationFactor(1.5f),
	alignmentFactor(0.8f),
	cohesionFactor(1.5f),
	maxSpeed(40.0f),
	minSpeed(30.0f),
	visionRadius(radius),
	visionAngle(visionAngle),
	grid(spatialGrid),
	boids(boidsStruct)
{ }

void Simulation::runSimulationFrame(float deltaTime)
{
	grid.updateBoidCellMap(boids, boids.boidsNumber);
	// Update the velocities based on the simulation rules
	for (int i = 0; i < boids.boidsNumber; i++)
	{
		std::vector<int> nhoodBoids = grid.getBoidsFromRegion(grid.hashBoid(boids, i));
		separation(nhoodBoids, i);
		cohesion(nhoodBoids, i);
		alignment(nhoodBoids, i);
	}
}

void Simulation::separation(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVecSum = 0.0f, yVecSum = 0.0f;
	
	for (auto boid : nhoodBoids)
	{
		if (isWithinFieldOfView(ownId, boid))
		{
			xVecSum += (boids.positionX[ownId] - boids.positionX[boid]);
			yVecSum += (boids.positionY[ownId] - boids.positionY[boid]);
		}
	}

	float norm = Helpers::vectorNorm(xVecSum, yVecSum);
	if (norm != 0)
	{
		xVecSum /= norm;
		yVecSum /= norm;
	}

	boids.updateVelocitySingle(xVecSum * separationFactor, yVecSum * separationFactor, ownId);
}

void Simulation::alignment(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVelSum = 0, yVelSum = 0;
	int boidCount = 0;
	// Average of the velocities
	for (auto boid : nhoodBoids)
	{
		if (isWithinFieldOfView(ownId, boid))
		{
			xVelSum += boids.velocityX[boid];
			yVelSum += boids.velocityY[boid];
			boidCount++;
		}
	}
	xVelSum /= boidCount;
	yVelSum /= boidCount;

	// Normalization
	float norm = Helpers::vectorNorm(xVelSum, yVelSum);
	if (norm != 0)
	{
		xVelSum /= norm;
		yVelSum /= norm;
	}

	boids.updateVelocitySingle((xVelSum - boids.velocityX[ownId]) * alignmentFactor,
		(yVelSum - boids.velocityY[ownId]) * alignmentFactor, ownId);
}


void Simulation::cohesion(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xPosAvg = 0.0f, yPosAvg = 0.0f;
	int boidsNum = 0;
	for (auto boid : nhoodBoids)
	{
		if (isWithinFieldOfView(ownId, boid))
		{
			xPosAvg += boids.positionX[boid];
			yPosAvg += boids.positionY[boid];
			boidsNum++;
		}
	}
	xPosAvg /= boidsNum;
	yPosAvg /= boidsNum;

	float xNewVelocity = xPosAvg - boids.positionX[ownId];
	float yNewVelocity = yPosAvg - boids.positionY[ownId];
	float norm = Helpers::vectorNorm(xNewVelocity, yNewVelocity);
	if (norm != 0)
	{
		xNewVelocity /= norm;
		yNewVelocity /= norm;
	}
	
	boids.updateVelocitySingle(xNewVelocity * cohesionFactor,
		yNewVelocity * cohesionFactor, ownId);
}


bool Simulation::isWithinRadius(int observerBoidId, int targetBoidId)
{
	if (observerBoidId < 0 || observerBoidId >= boids.boidsNumber ||
		targetBoidId < 0 || targetBoidId >= boids.boidsNumber) {
		return false;
	}
	float xDistance = std::abs(boids.positionX[observerBoidId] - boids.positionX[targetBoidId]);
	float yDistance = std::abs(boids.positionY[observerBoidId] - boids.positionY[targetBoidId]);
	float distance = std::sqrt(xDistance * xDistance + yDistance * yDistance);

	return distance < visionRadius;

}

bool Simulation::isWithinVisionCone(int observerBoid, int targetBoid)
{
	// Direction of observer boid is the same as its velocity direction
	
	// Vector from observer boid to the target boid
	float xDistanceVec = boids.positionX[targetBoid] - boids.positionY[observerBoid];
	float yDistanceVec = boids.positionY[targetBoid] - boids.positionY[observerBoid];

	float dotProduct = xDistanceVec * boids.velocityX[observerBoid]
		+ yDistanceVec * boids.velocityY[observerBoid];
	float determinant = xDistanceVec * boids.velocityY[observerBoid]
		- yDistanceVec * boids.velocityX[observerBoid];
	
	// Angle between the two boids
	float angle = fabs(atan2f(determinant, dotProduct));

	if (angle <= visionAngle)
		return true;
	return false;
}

bool Simulation::isWithinFieldOfView(int observerBoid, int targetBoid)
{
	return (isWithinRadius(observerBoid, targetBoid) 
		&& isWithinVisionCone(observerBoid, targetBoid));
}
