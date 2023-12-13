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

        velocityX[i] = randX * 50;
        velocityY[i] = randY * 50;
    }
}

void Boids::updatePositionsSingleBoid(float deltaTime, int boidId, int screenWidth, int screenHeight)
{
    // For a single boid
    float changePosX = velocityX[boidId] * deltaTime;
    if (positionX[boidId] + changePosX > screenWidth)
    {
        // position out of the window to the right
        positionX[boidId] = screenWidth - 0.1f;
        velocityX[boidId] = -velocityX[boidId];
    }
    else if (positionX[boidId] + changePosX < 0)
    {
        // posiiton out of the window to the left
        positionX[boidId] = 0.1f;
        velocityX[boidId] = -velocityX[boidId];
    }
    else
    {
        positionX[boidId] += changePosX;
    }

    float changePosY = velocityY[boidId] * deltaTime;
    if (positionY[boidId] + changePosY > screenHeight)
    {
        // position out of the window to the right
        positionY[boidId] = screenHeight - 0.1f;
        velocityY[boidId] = -velocityY[boidId];
    }
    else if (positionY[boidId] + changePosY < 0)
    {
        // posiiton out of the window to the left
        positionY[boidId] = 0.1f;
        velocityY[boidId] = -velocityY[boidId];
    }
    else
    {
        positionY[boidId] += changePosY;
    }
}

void Boids::updatePositionsAllBoids(float deltaTime, int screenWidth, int screenHeight)
{
    for (int i = 0; i < boidsNumber; i++)
    {
        updatePositionsSingleBoid(deltaTime, i, screenWidth, screenHeight);
    }
}
