#ifndef CAR_HPP
#define CAR_HPP

#include <cmath>

const double MPH_TO_MPS = 0.44704;

struct Car
{
  Car(double xv,
      double yv,
      double sv,
      double dv,
      double yawv,
      double speedv) :
      x{xv}, y{yv}, s{sv}, d{dv}, yaw{yawv}, speed{speedv*MPH_TO_MPS}
  {}

  inline unsigned int lane() const
  {
    return static_cast<int>(std::floor(d / 4.0));
  }

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

#endif
