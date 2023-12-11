#pragma once
#include <vector>

struct Boids
{
	int boidsNumber;
	std::vector<float> positionX;
	std::vector<float> positionY;
	std::vector<float> velocityX;
	std::vector<float> velocityY;

	Boids(int boidsNumber);
	void randomizeParameters(int screenWidth, int screenHeight);
};