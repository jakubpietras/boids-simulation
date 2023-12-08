#include <iostream>
#include "SpatialGrid.h"
#include "Boids.h"
#include "Renderer.h"
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <vector>

int main(void)
{
	const int boidsNumber = 100;
	const int nbhoodRadius = 25;
	const int screenWidth = 800;
	const int screenHeight = 600;

	std::srand(std::time(nullptr));

	Boids boids = Boids(boidsNumber);
	boids.randomizeParameters(screenWidth, screenHeight);

	auto start = std::chrono::high_resolution_clock::now();
	SpatialGrid grid = SpatialGrid(screenWidth, screenHeight, nbhoodRadius, boidsNumber);
	grid.updateBoidCellMap(boids, boidsNumber);


	auto list = grid.getBoidsFromRegion(grid.hashBoid(boids, 4));
	auto stop = std::chrono::high_resolution_clock::now();
	for(auto item : list)
	{
		std::cout << item << std::endl;
	}

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Time: " << duration.count() << std::endl;

	Renderer r = Renderer(screenWidth, screenHeight);
	r.render();
	

	return 0;
}