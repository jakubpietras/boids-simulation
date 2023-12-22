#include "GUIController.h"
#include "Renderer.h"
#include "SpatialGrid.h"
#include "Boids.h"
#include "Simulation.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <vector>

int main(void)
{
	// Initial parameters of the simulation
	const int boidsNumber = 5000;
	const float nbhoodRadius = 10.0f;
	const float visionAngle = 110.0f;
	const int screenWidth = 1600;
	const int screenHeight = 880;
	const int minSpeed = 50;
	const int maxSpeed = 70;
	float modelVertices[] = {
	-0.25f, -0.5f, 0.0f,
	 0.25f, -0.5f, 0.0f,
	 0.0f,  0.5f, 0.0f
	};
	const char* vertexShaderPath = "./shaders/boid.vert";
	const char* fragmentShaderPath = "./shaders/boid.frag";

	std::srand(std::time(nullptr));

	Boids boids = Boids(boidsNumber, minSpeed, maxSpeed);
	boids.randomizeParameters(screenWidth, screenHeight);

	float cellSize = 2 * nbhoodRadius;
	SpatialGrid grid = SpatialGrid(screenWidth, screenHeight, cellSize, boidsNumber);
	grid.updateBoidCellMap(boids, boidsNumber);

	Renderer r(screenWidth, screenHeight);
	Shader sh(vertexShaderPath, fragmentShaderPath);
	Simulation sim(boids, grid, nbhoodRadius, visionAngle);
	GUIController gui(r.window, sim, cellSize);

	float deltaTime = 0.0f;
	while (!glfwWindowShouldClose(r.window))
	{
		r.renderFrame(boids, modelVertices, sh, sim, deltaTime, gui);
		boids.updatePositionsAllBoids(deltaTime, screenWidth, screenHeight);
		sim.runSimulationFrame(deltaTime);
	}
	gui.terminate();
	glfwTerminate();
	
	return 0;
}

