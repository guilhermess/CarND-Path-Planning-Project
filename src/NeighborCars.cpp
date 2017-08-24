
#include "NeighborCars.hpp"
#include <cmath>

pair<double,double> NeighborCar::estimate_car_frenet_position(unsigned int interval_seconds) const
{
  double total_velocity = std::sqrt(vx_*vx_ + vy_*vy_);
  double estimated_s = s_ + total_velocity * interval_seconds;
  return std::make_pair(estimated_s, d_);
}