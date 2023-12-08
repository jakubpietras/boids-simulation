#include "BoidCellMap.h"
#include <algorithm>

BoidCellMap::BoidCellMap()
	: size(0)
{
	// dummy data
	data.push_back(MapCell(0, 0));
}

BoidCellMap::BoidCellMap(int boidsNumber)
	: size(boidsNumber)
{
	for (int i = 0; i < size; i++)
	{
		data.push_back(MapCell(0, 0));
	}
}

void BoidCellMap::sortMap()
{
	std::sort(data.begin(), data.end(), [](const MapCell& a, const MapCell& b) 
	{
		return a.cellNumber < b.cellNumber;
	});
}
