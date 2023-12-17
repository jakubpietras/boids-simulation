#pragma once
//#include "SpatialGrid.h"
#include "Helpers.h"
#include <vector>

struct Boids
{
	int boidsNumber;
	std::vector<float> positionX, positionY, velocityX, velocityY;
	float minSpeed, maxSpeed;

	Boids(int boidsNumber, float minSpeed, float maxSpeed);
	void randomizeParameters(int screenWidth, int screenHeight);
	void updatePositionsSingleBoid(float deltaTime, int boidId, int screenWidth, int screenHeight);
	void updatePositionsAllBoids(float deltaTime, int screenWidth, int screenHeight);
	void updateVelocitySingle(float velocityChangeX, float velocityChangeY, int boidId);
};