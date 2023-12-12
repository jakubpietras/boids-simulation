#include <iostream>
#include "SpatialGrid.h"
#include "Boids.h"
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <vector>
#include "Renderer.h"

int main(void)
{
	const int boidsNumber = 50;
	const int nbhoodRadius = 25;
	const int screenWidth = 800;
	const int screenHeight = 600;

	const char* vertexShaderPath = "./boid.vert";
	const char* fragmentShaderPath = "./boid.frag";

	float vertices[] = {
		-0.5f, -0.5f, 0.0f, // left  
		 0.5f, -0.5f, 0.0f, // right 
		 0.0f,  0.5f, 0.0f  // top   
	};


	std::srand(std::time(nullptr));

	Boids boids = Boids(boidsNumber);
	boids.randomizeParameters(screenWidth, screenHeight);

	auto start = std::chrono::high_resolution_clock::now();
	SpatialGrid grid = SpatialGrid(screenWidth, screenHeight, nbhoodRadius, boidsNumber);
	grid.updateBoidCellMap(boids, boidsNumber);


	auto list = grid.getBoidsFromRegion(grid.hashBoid(boids, 2));
	auto stop = std::chrono::high_resolution_clock::now();
	for(auto item : list)
	{
		std::cout << item << std::endl;
	}

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Time: " << duration.count() << std::endl;
	Renderer r(screenWidth, screenHeight);
	Shader sh(vertexShaderPath, fragmentShaderPath);

	r.render(vertices, sh);

	return 0;
}