#pragma once
#include <utility>
#include <vector>
#include <array>
#include <cmath>
#include <iostream>
#include "Boids.h"
#include "BoidCellMap.h"

class SpatialGrid
{
private:
	int totalWidth, totalHeight, cellsNumberX, cellsNumberY,
		cellsNumber, cellSize, boidsNumber;
	std::vector<int> cellsStart;
	BoidCellMap boidCellMap;

public:
	SpatialGrid(int width, int height, int cellSize, int boidsNumber);
	int hashBoid(Boids& boids, int boidId);
	void updateBoidCellMap(Boids& boids, int boidsNumber);
	void updateCellsStart();
	int cellRowIndex(int cellIndex);
	bool isCellInRow(int cellIndex, int rowIndex);
	bool isCellInGrid(int cellIndex);
	std::vector<int> getBoidsFromCell(int cellIndex);
	std::vector<int> getBoidsFromRegion(int centerCellIndex);
};