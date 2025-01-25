#include "timer.h"

Timer::Timer()
    : deltaTime(0.0f), fps(0.0f), frameCount(0), elapsedTime(0.0f)
{
    lastTime = std::chrono::high_resolution_clock::now();
}

void Timer::update()
{
    auto currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> frameDuration = currentTime - lastTime;
    deltaTime = frameDuration.count();
    lastTime = currentTime;

    frameCount++;
    elapsedTime += deltaTime;

    if (elapsedTime >= 1.0f / 2)
    {
        fps = static_cast<float>(frameCount) / elapsedTime;
        frameCount = 0;
        elapsedTime = 0.0f;
    }
}

float Timer::getDeltaTime()
{
	return deltaTime;
}

float Timer::getFPS()
{
	return fps;
}
