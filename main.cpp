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
	const int boidsNumber = 1000;
	const int nbhoodRadius = 10;
	const int screenWidth = 1600;
	const int screenHeight = 880;
	const int minSpeed = 30;
	const int maxSpeed = 50;
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

	SpatialGrid grid = SpatialGrid(screenWidth, screenHeight, nbhoodRadius, boidsNumber);
	grid.updateBoidCellMap(boids, boidsNumber);

	Renderer r(screenWidth, screenHeight);
	Shader sh(vertexShaderPath, fragmentShaderPath);
	Simulation sim(boids, grid, nbhoodRadius);
	GUIController gui(r.window, sim);

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

