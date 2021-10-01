
#include "MathUtils.h"
#include <Arduino.h>
bool equalish(double x, double y)
{
        return (abs(x - y) < 0.5);
}