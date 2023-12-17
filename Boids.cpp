#include "Boids.h"
#include "SpatialGrid.h"
#include "Helpers.h"


Boids::Boids(int boidsNumber, float minSpeed, float maxSpeed)
    : boidsNumber(boidsNumber),
    positionX(boidsNumber, 0.0f),
    positionY(boidsNumber, 0.0f),
    velocityX(boidsNumber, 0.0f),
    velocityY(boidsNumber, 0.0f),
    minSpeed(minSpeed),
    maxSpeed(maxSpeed)
{ }

void Boids::randomizeParameters(int screenWidth, int screenHeight)
{
    for (int i = 0; i < boidsNumber; i++)
    {        
        positionX[i] = Helpers::randomFloat(0.0f, static_cast<float>(screenWidth)/2);
        positionY[i] = Helpers::randomFloat(0.0f, static_cast<float>(screenHeight)/2);
        velocityX[i] = Helpers::randomFloat(-maxSpeed, maxSpeed);
        velocityY[i] = Helpers::randomFloat(-maxSpeed, maxSpeed);
    }
}

void Boids::updatePositionsSingleBoid(float deltaTime, int boidId, int screenWidth, int screenHeight)
{
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
        // position above the top of the window
        positionY[boidId] = screenHeight - 0.1f;
        velocityY[boidId] = -velocityY[boidId];
    }
    else if (positionY[boidId] + changePosY < 0)
    {
        // position below the bottom of the window
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

void Boids::updateVelocitySingle(float velocityChangeX, float velocityChangeY, int boidId)
{
    float xNewVelocity = velocityX[boidId] + velocityChangeX;
    float yNewVelocity = velocityY[boidId] + velocityChangeY;

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

    velocityX[boidId] = xNewVelocity;
    velocityY[boidId] = yNewVelocity;
}
