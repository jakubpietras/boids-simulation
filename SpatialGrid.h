#pragma once
#include <utility>
#include <vector>
#include <array>
#include "Boids.h"
#include "BoidCellMap.h"

class SpatialGrid
{
private:
	int totalWidth;
	int totalHeight;
	int cellsNumberX;
	int cellsNumberY;
	int cellsNumber;
	int cellSize;
	std::vector<int> cellsStart;
	int boidsNumber;
	BoidCellMap boidCellMap;


public:
	SpatialGrid(int width, int height, int cellSize, int boidsNumber);
	int hashBoid(Boids& boids, int boidId);
	void updateBoidCellMap(Boids& boids, int boidsNumber);
	void updateCellsStart();
	std::vector<int> getBoidsFromCell(int cellIndex);
	std::vector<int> getBoidsFromRegion(int centerCellIndex);
	void printGibberish();
};