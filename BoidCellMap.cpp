#include "BoidCellMap.h"

BoidCellMap::BoidCellMap() = default;

BoidCellMap::BoidCellMap(int boidsNumber)
	: size(boidsNumber), data(boidsNumber, MapCell(0, 0))
{ }

void BoidCellMap::sortMap()
{
	std::sort(data.begin(), data.end(), [](const MapCell& a, const MapCell& b) 
	{
		return a.cellNumber < b.cellNumber;
	});
}
