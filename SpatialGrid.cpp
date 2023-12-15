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

int SpatialGrid::cellRowIndex(int cellIndex)
{
	int rowIndex = cellIndex / cellsNumberX;
	return (rowIndex >= cellsNumberX || cellIndex < 0) ? -1 : rowIndex;
}

bool SpatialGrid::isCellInRow(int cellIndex, int rowIndex)
{
	return cellRowIndex(cellIndex) == rowIndex;
}

bool SpatialGrid::isCellInGrid(int cellIndex)
{
	return cellIndex >= 0 && cellIndex < cellsNumber;
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

std::vector<int> SpatialGrid::getBoidsFromRegion2(int centerCellIndex)
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

std::vector<int> SpatialGrid::getBoidsFromRegion3(int centerCellIndex)
{
	std::vector<int> boidsIndices;
	std::vector<int> boidsToAdd;
		
	// Center cell
	boidsToAdd = getBoidsFromCell(centerCellIndex);
	boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());

	if (cellRowIndex(centerCellIndex + 1) == cellRowIndex(centerCellIndex) && (centerCellIndex + 1 < cellsNumber))
	{
		boidsToAdd = getBoidsFromCell(centerCellIndex + 1);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	if (cellRowIndex(centerCellIndex - 1) == cellRowIndex(centerCellIndex) && (centerCellIndex - 1 > 0))
	{
		boidsToAdd = getBoidsFromCell(centerCellIndex - 1);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}

	// Upper row
	int topCellIndex = centerCellIndex + cellsNumberX;
	if (cellRowIndex(topCellIndex) != -1)
	{
		boidsToAdd = getBoidsFromCell(topCellIndex);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());

		if (cellRowIndex(topCellIndex + 1) == cellRowIndex(topCellIndex) && (topCellIndex + 1 < cellsNumber))
		{
			boidsToAdd = getBoidsFromCell(topCellIndex + 1);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
		if (cellRowIndex(topCellIndex - 1) == cellRowIndex(topCellIndex))
		{
			boidsToAdd = getBoidsFromCell(topCellIndex - 1);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
	}

	// Bottom row
	int bottomCellIndex = centerCellIndex - cellsNumberX;
	if (cellRowIndex(bottomCellIndex) != -1)
	{
		boidsToAdd = getBoidsFromCell(bottomCellIndex);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());

		if (cellRowIndex(bottomCellIndex + 1) == cellRowIndex(bottomCellIndex))
		{
			boidsToAdd = getBoidsFromCell(bottomCellIndex + 1);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
		if (cellRowIndex(bottomCellIndex - 1) == cellRowIndex(bottomCellIndex) && (bottomCellIndex - 1 > 0))
		{
			boidsToAdd = getBoidsFromCell(bottomCellIndex - 1);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
	}

	return boidsIndices;
}

std::vector<int> SpatialGrid::getBoidsFromRegion(int centerCellIndex)
{
	std::vector<int> boidsIndices;
	std::vector<int> boidsToAdd;

	int center = centerCellIndex, 
		left = centerCellIndex - 1, 
		right = centerCellIndex - 1,
		top = centerCellIndex + cellsNumberX,
		topLeft = centerCellIndex + cellsNumberX - 1,
		topRight = centerCellIndex + cellsNumberX + 1,
		bottom = centerCellIndex - cellsNumberX,
		bottomLeft = centerCellIndex - cellsNumberX - 1,
		bottomRight = centerCellIndex - cellsNumberX + 1;

	boidsToAdd = getBoidsFromCell(center);
	boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());

	if (isCellInRow(left, cellRowIndex(center)) && isCellInGrid(left))
	{
		boidsToAdd = getBoidsFromCell(left);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	if (isCellInRow(right, cellRowIndex(center)) && isCellInGrid(right))
	{
		boidsToAdd = getBoidsFromCell(right);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
	}
	
	if (isCellInGrid(top))
	{
		boidsToAdd = getBoidsFromCell(top);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		if (isCellInRow(topLeft, cellRowIndex(top)) && isCellInGrid(topLeft))
		{
			boidsToAdd = getBoidsFromCell(topLeft);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
		if (isCellInRow(topRight, cellRowIndex(top)) && isCellInGrid(topRight))
		{
			boidsToAdd = getBoidsFromCell(topRight);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
	}

	if (isCellInGrid(bottom))
	{
		boidsToAdd = getBoidsFromCell(bottom);
		boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		if (isCellInRow(bottomLeft, cellRowIndex(bottom)) && isCellInGrid(bottomLeft))
		{
			boidsToAdd = getBoidsFromCell(bottomLeft);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
		if (isCellInRow(bottomRight, cellRowIndex(bottom)) && isCellInGrid(bottomRight))
		{
			boidsToAdd = getBoidsFromCell(bottomRight);
			boidsIndices.insert(boidsIndices.end(), boidsToAdd.begin(), boidsToAdd.end());
		}
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