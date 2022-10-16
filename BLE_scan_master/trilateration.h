#ifndef TRILATERATION_H
#define TRILATERATION_H

#define RAD_TO_DEG 57.2957786

struct point3D {
  double x;
  double y;
  double z;
};

struct pointCartesian2D {
  double x;
  double y;
};

struct pointPolar2D {
  double r;
  double phi;
};

//assumptions about receivers locations:
//C1(0,0,0)
//C2(U,0,0)
//C3(Vx, Vy, 0)
point3D trilaterate3DspheresInPlane(double r1, double r2, double r3, 
                                    double U = 1, double Vx = 0, double Vy = 1);
pointPolar2D cart2pol(pointCartesian2D cartP);

#endif
