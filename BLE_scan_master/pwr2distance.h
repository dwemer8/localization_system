#ifndef PWR2DISTANCE_H
#define PWR2DISTANCE_H

#include "math.h"

//P_rx = P_tx - c1' -c2'*ln(d/lambda) + sigma^2
//P_rx = -c1 - c2*ln(d/lambda)
//d = lambda*exp(-(P_rx + c1)/c2)
#define lambda 0.125 //bluetooth wavelength responding to 2.4 GHz, measurements calibrated for this number
#define c1 38.88561
#define c2 5.95554

double pwr2dist(double pwr);

#endif
