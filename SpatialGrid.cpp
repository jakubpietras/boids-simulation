#include <iostream>
#include "SpatialGrid.h"
#include "Boids.h"
#include <cmath>

SpatialGrid::SpatialGrid(int width, int height, int cellSize, int boidsNumber)
	: totalWidth(width), totalHeight(height), cellSize(cellSize), boidsNumber(boidsNumber)
{
	cellsNumberX = totalWidth / cellSize;
	cellsNumberY = totalHeight / cellSize;
	cellsNumber = cellsNumberX * cellsNumberY;
	cellsStart = std::vector<int>(cellsNumber, -1);
	boidCellMap = BoidCellMap(boidsNumber);
}


int SpatialGrid::hashBoid(Boids& boids, int boidId)
{
	int cellIdX = static_cast<int>(floor(boids.positionX[boidId] / cellSize));
	int cellIdY = static_cast<int>(floor(boids.positionY[boidId] / cellSize));
	//// Debugging output
	//std::cout << "Boid: " << boidId << ", Position: (" << boids.positionX[boidId] << ", " << boids.positionY[boidId] << "), Cell: (" << cellIdX << ", " << cellIdY << ")\n";

	return cellIdY * cellsNumberX + cellIdX;
}

void SpatialGrid::updateBoidCellMap(Boids& boids, int boidsNumber)
{
	for (int i = 0; i < boidsNumber; i++)
	{
		boidCellMap.data[i].boidNumber = i;
		boidCellMap.data[i].cellNumber = hashBoid(boids, i);
	}
	boidCellMap.sortMap();
	updateCellsStart();
}

void SpatialGrid::updateCellsStart()
{
	for (int i = 1; i < boidsNumber; i++)
	{
		if (boidCellMap.data[i].cellNumber != boidCellMap.data[i - 1].cellNumber)
		{
			cellsStart[boidCellMap.data[i].cellNumber] = i;
		}
	}
	cellsStart[boidCellMap.data[0].cellNumber] = 0;
}

std::vector<int> SpatialGrid::getBoidsFromCell(int cellIndex)
{
	std::vector<int> boidsIndices;
	int i = cellsStart[cellIndex];
	if (i < 0)
		return boidsIndices;
	while (i < boidsNumber && boidCellMap.data[i].cellNumber == cellIndex)
	{
		boidsIndices.push_back(boidCellMap.data[i].boidNumber);
		i++;
	}
	return boidsIndices;
}

std::vector<int> SpatialGrid::getBoidsFromRegion(int centerCellIndex)
{
	std::vector<int> boidsIndices;
	// Checking center cell
	std::vector<int> boidsToAdd = getBoidsFromCell(centerCellIndex);
	boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	// Checking top cell
	if (centerCellIndex + cellsNumberX < cellsNumber)
	{
		boidsToAdd = getBoidsFromCell(centerCellIndex + cellsNumberX);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	if (centerCellIndex % cellsNumber == 0)		// Cell at the left boundary
		return boidsIndices;
	// Checking left cell
	if (centerCellIndex - 1 > 0)
	{
		boidsToAdd = getBoidsFromCell(centerCellIndex - 1);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	// Checking top-left cell
	if (centerCellIndex + cellsNumberX - 1 < cellsNumber)
	{
		boidsToAdd = getBoidsFromCell(centerCellIndex + cellsNumberX - 1);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	return boidsIndices;
}

void SpatialGrid::printGibberish()
{
	for (int i = 0; i < boidsNumber; i++)
	{
		std::cout << "B: " << boidCellMap.data[i].boidNumber << " C: " << boidCellMap.data[i].cellNumber << std::endl;
	}
}