#include "trilateration.h"
#include "math.h"

point3D trilaterate3DspheresInPlane(double r1, double r2, double r3, 
                                  double U, double Vx, double Vy) {
  point3D M;
  M.x = (r1*r1 - r2*r2 + U*U)/(2*U);
  M.y = (r1*r1 - r3*r3 + Vx*Vx + Vy*Vy + 2*Vx*M.x)/(2*Vy);
  M.z = r1*r1 - M.x*M.x - M.y*M.y;
  if (M.z <= 0) {
    M.z = 0;
  } else {
    M.z = sqrt(M.z);
  }
  return M;
}

pointPolar2D cart2pol(pointCartesian2D cartP) {
  pointPolar2D polP;
  polP.r = hypot(cartP.x, cartP.y);
  polP.phi = atan2(cartP.y, cartP.x) * RAD_TO_DEG;
  return polP;
}
