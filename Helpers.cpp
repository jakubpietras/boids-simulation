#include "Helpers.h"

float Helpers::randomFloat(float minValue, float maxValue)
{
    float random = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return minValue + random * (maxValue - minValue);
}
