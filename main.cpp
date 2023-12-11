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
	const int boidsNumber = 50;
	const int nbhoodRadius = 25;
	const int screenWidth = 800;
	const int screenHeight = 600;

	const char* vertexShader =
		"#version 330 core\n"
		"layout (location = 0) in float inPositionX;\n"
		"layout (location = 1) in float inPositionY;\n"
		"void main()\n"
		"{\n"
		"	vec4 localPosition = vec4(inPositionX, inPositionY, 0.0, 1.0)\n"
		"	\n"
		"   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
		"}\0";
	const char* fragmentShader =
		"#version 330 core\n"
		"out vec4 FragColor;\n"
		"void main()\n"
		"{\n"
		"	FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f)\n"
		"}\0";

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

	Renderer r = Renderer(screenWidth, screenHeight);
	r.render();
	

	return 0;
}