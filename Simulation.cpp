#include "Simulation.h"
#include "Boids.h"
#include "SpatialGrid.h"

Simulation::Simulation(Boids& boidsStruct, SpatialGrid& spatialGrid, float radius)
	: separationFactor(0.5f),
	alignmentFactor(0.5f),
	cohesionFactor(0.05f),
	maxSpeed(40.0f),
	minSpeed(30.0f),
	visionRadius(radius),
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
		//std::vector<int> boidsInRadius;
		//std::copy_if(nhoodBoids.begin(), nhoodBoids.end(), std::back_inserter(boidsInRadius), [i, this](int boid) {return isWithinRadius(i, boid, visionRadius); });
		separation(nhoodBoids, i);
		alignment(nhoodBoids, i);
		cohesion(nhoodBoids, i);
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
	boids.updateVelocitySingle(xVecSum * separationFactor, yVecSum * separationFactor, ownId);
}

void Simulation::alignment(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVecSum = 0, yVecSum = 0;
	for (auto boid : nhoodBoids)
	{
		if (isWithinFieldOfView(ownId, boid))
		{
			xVecSum += (boids.positionX[ownId] - boids.positionX[boid]);
			yVecSum += (boids.positionY[ownId] - boids.positionY[boid]);
		}
	}
	xVecSum *= alignmentFactor;
	yVecSum *= alignmentFactor;

	boids.updateVelocitySingle(boids.velocityX[ownId] - xVecSum,
		boids.velocityY[ownId] - yVecSum, ownId);
}

/* void Simulation::alignment2(std::vector<int> nhoodBoids, int ownId)
{
	if (nhoodBoids.size() == 0)
		return;

	float xVecSum = 0, yVecSum = 0;
	int boidsNum = 0;
	for (auto boid : nhoodBoids)
	{
		if (isWithinFieldOfView(ownId, boid))
		{
			xVecSum += (boids.positionX[ownId] - boids.positionX[boid]);
			yVecSum += (boids.positionY[ownId] - boids.positionY[boid]);
			boidsNum++;
		}
	}

	xVecSum *= alignmentFactor / boidsNum;
	yVecSum *= alignmentFactor / boidsNum;

	boids.updateVelocitySingle(((xVecSum/boidsNum - boids.velocityX[ownId]) * alignmentFactor),
		((yVecSum / boidsNum - boids.velocityY[ownId]) * alignmentFactor), ownId);
}
*/

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

	boids.updateVelocitySingle((xPosAvg - boids.positionX[ownId]) * cohesionFactor,
		(yPosAvg - boids.positionY[ownId]) * cohesionFactor, ownId);

	/*boids.velocityX[ownId] += (xPosAvg - boids.positionX[ownId]) * cohesionFactor;
	boids.velocityY[ownId] += (yPosAvg - boids.positionY[ownId]) * cohesionFactor;*/
}

//void Simulation::updateVelocity(float xVelocity, float yVelocity, int boidId)
//{
//	float xNewVelocity = boids.velocityX[boidId] + xVelocity;
//	float yNewVelocity = boids.velocityY[boidId] + yVelocity;
//
//	float newSpeed = sqrt(xNewVelocity * xNewVelocity + yNewVelocity * yNewVelocity);
//
//	if (newSpeed > maxSpeed)
//	{
//		xNewVelocity *= (maxSpeed / newSpeed);
//		yNewVelocity *= (maxSpeed / newSpeed);
//	}
//	if (newSpeed < minSpeed)
//	{
//		xNewVelocity *= (minSpeed / newSpeed);
//		yNewVelocity *= (minSpeed / newSpeed);
//	}
//
//	boids.velocityX[boidId] = xNewVelocity;
//	boids.velocityY[boidId] = yNewVelocity;
//}

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

//bool Simulation::isWithinConeOfVision(int observerBoidId, int targetBoidId)
//{
//	if (observerBoidId < 0 || observerBoidId >= boids.boidsNumber ||
//		targetBoidId < 0 || targetBoidId >= boids.boidsNumber) {
//		return false;
//	}
//
//	// Get velocity vector of the observer boid
//	float observerVelX = boids.velocityX[observerBoidId];
//	float observerVelY = boids.velocityY[observerBoidId];
//
//	// Get vector from observer to target
//	float toTargetX = boids.positionX[targetBoidId] - boids.positionX[observerBoidId];
//	float toTargetY = boids.positionY[targetBoidId] - boids.positionY[observerBoidId];
//
//	// Calculate dot product
//	float dotProduct = observerVelX * toTargetX + observerVelY * toTargetY;
//
//	// Calculate cosine of the cone angle
//	float cosConeAngle = cos(visionAngle);
//
//	// Compare with the cosine of the cone angle
//	return dotProduct > cosConeAngle;
//}

bool Simulation::isWithinFieldOfView(int observerBoidId, int targetBoidId)
{
	return (isWithinRadius(observerBoidId, targetBoidId));
}
