
#include "MathUtils.h"
#include <Arduino.h>
bool equalish(double x, double y)
{
        return (abs(x - y) < 0.5);
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
