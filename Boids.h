#pragma once

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
};