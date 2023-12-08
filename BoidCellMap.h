#pragma once
#include <vector>

struct MapCell
{
	int boidNumber;
	int cellNumber;
	MapCell(int boidNum, int cellNum) 
		: boidNumber(boidNum), cellNumber(cellNum) {}
};

struct BoidCellMap
{
	std::vector<MapCell> data;
	int size;

	BoidCellMap();
	BoidCellMap(int boidsNumber);
	void sortMap();
};