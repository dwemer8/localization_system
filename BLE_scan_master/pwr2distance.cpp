#include "pwr2distance.h"

//P_rx = P_tx - c1' -c2'*ln(d/lambda) + sigma^2
//P_rx = -c1 - c2*ln(d/lambda)
//d = lambda*exp(-(P_rx + c1)/c2)
double pwr2dist(double pwr) {
  return lambda*exp(-(pwr + c1)/c2); 
}
