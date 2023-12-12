#include "Boids.h"
#include "SpatialGrid.h"
#include <cstdlib>


Boids::Boids(int boidsNumber)
    : boidsNumber(boidsNumber)
{
    positionX = new float[boidsNumber];
    positionY = new float[boidsNumber];
    velocityX = new float[boidsNumber];
    velocityY = new float[boidsNumber];
}

Boids::~Boids()
{
    delete[] positionX;
    delete[] positionY;
    delete[] velocityX;
    delete[] velocityY;
}

void Boids::randomizeParameters(int screenWidth, int screenHeight)
{
    for (int i = 0; i < boidsNumber; i++)
    {
        // Generate random values in the range [0, 1)
        float randX = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float randY = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // Scale and shift to the desired range
        positionX[i] = randX * screenWidth;
        positionY[i] = randY * screenHeight;

        velocityX[i] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        velocityY[i] = 1 - velocityX[i];
    }
}