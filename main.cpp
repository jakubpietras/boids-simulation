#include <iostream>
#include "SpatialGrid.h"
#include "Boids.h"
#include "Simulation.h"
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <vector>
#include "Renderer.h"

int main(void)
{
	const int boidsNumber = 1000;
	const int nbhoodRadius = 20;
	const int screenWidth = 1920;
	const int screenHeight = 1080;

	const char* vertexShaderPath = "./shaders/boid.vert";
	const char* fragmentShaderPath = "./shaders/boid.frag";

	float vertices[] = {
	-0.25f, -0.5f, 0.0f,
	 0.25f, -0.5f, 0.0f,
	 0.0f,  0.5f, 0.0f
	};


	std::srand(std::time(nullptr));

	Boids boids = Boids(boidsNumber);
	boids.randomizeParameters(screenWidth, screenHeight);

	auto start = std::chrono::high_resolution_clock::now();
	SpatialGrid grid = SpatialGrid(screenWidth, screenHeight, nbhoodRadius, boidsNumber);
	grid.updateBoidCellMap(boids, boidsNumber);


	auto list = grid.getBoidsFromRegion(grid.hashBoid(boids, 0));
	auto stop = std::chrono::high_resolution_clock::now();
	for(auto item : list)
	{
		std::cout << item << std::endl;
	}

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Time: " << duration.count() << std::endl;
	Renderer r(screenWidth, screenHeight);
	Shader sh(vertexShaderPath, fragmentShaderPath);
	Simulation sim(boids, grid);

	r.render(boids, vertices, sh, sim);

	return 0;
}