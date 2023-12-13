#pragma once
#include <vector>

struct Boids
{
	int boidsNumber;
	float* positionX;
	float* positionY;
	float* velocityX;
	float* velocityY;

	Boids(int boidsNumber);
	~Boids();
	void randomizeParameters(int screenWidth, int screenHeight);
	void updatePositionsSingleBoid(float deltaTime, int boidId, int screenWidth, int screenHeight);
	void updatePositionsAllBoids(float deltaTime, int screenWidth, int screenHeight);
};