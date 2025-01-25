#ifndef SIMULATION_STATE_H
#define SIMULATION_STATE_H
#include "mode.h"

struct SimulationState
{
	Mode currentMode;
	float separationFactor, 
		alignmentFactor, 
		cohesionFactor,
		visionRadius,
		visionAngle,
		maxSpeed,
		boxSize;
	bool isPlaying, resetBoids;
};


#endif