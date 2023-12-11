#include "Boids.h"
#include "SpatialGrid.h"
#include <cstdlib>


Boids::Boids(int boidsNumber)
    : boidsNumber(boidsNumber),
    positionX(boidsNumber, 0.0f),
    positionY(boidsNumber, 0.0f),
    velocityX(boidsNumber, 0.0f),
    velocityY(boidsNumber, 0.0f) { }

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

        velocityX[i] = 0;
        velocityY[i] = 0;
    }
}
