#ifndef TIMER_H
#define TIMER_H
#include <chrono>

class Timer {
public:
	Timer();
	void update();
	float getDeltaTime();
	float getFPS();
private:
	std::chrono::high_resolution_clock::time_point startTime, lastTime;
	float deltaTime, elapsedTime, fps;
	int frameCount;
};

#endif //TIMER_H