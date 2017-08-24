
#include "SpeedEstimation.hpp"
#include <algorithm>
#include <iostream>

pair<double,double> SpeedEstimation::estimate_speed_acceleration(double distance_to_car,
                                                    double current_speed_mps,
                                                    double target_distance_to_car) const
{
  int num_steps = interval_seconds_ * 1000 / dt_miliseconds_;

  double delta_distance = distance_to_car - target_distance_to_car;
  double speed_meters_per_second = current_speed_mps;

  double accel_step = 0.5;
  for ( double accel = max_acceleration_ms2_; accel >= -1 * max_acceleration_ms2_; accel -= 0.5 )
  {
    double distance = delta_distance;
    double speed = speed_meters_per_second;

    for (int i = 0; i < num_steps; ++i)
    {
      speed += accel / num_steps;
      speed = std::min(speed, target_speed_meters_per_second_);
      distance -= (speed * dt_miliseconds_ * 1e-3);
    }

    if ( distance >= 0 )
    {
      return std::make_pair(speed, accel);
    }
  }

  double speed = speed_meters_per_second;
  double accel = max_acceleration_ms2_ * -1;
  for (int i = 0; i < num_steps; ++i)
  {
    speed += accel / num_steps;
    speed = std::min(speed, target_speed_meters_per_second_);
    speed = std::max(speed, 0.0);
  }
  return std::make_pair(speed, accel);

}