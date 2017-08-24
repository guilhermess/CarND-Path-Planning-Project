#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "Map.hpp"
#include "Car.hpp"
#include "Path.hpp"

class TrajectoryGenerator
{
 public:
  TrajectoryGenerator(const Map &map,
                      double target_speed_meters_per_second,
                      double max_acceleration_ms2,
                      double min_distance_to_neighbor,
                      unsigned int dt_miliseconds,
                      unsigned int interval_seconds) : map_{map},
                                              target_speed_meters_per_second_{target_speed_meters_per_second},
                                              max_acceleration_ms2_{max_acceleration_ms2},
                                              min_distance_to_neighbor_{min_distance_to_neighbor},
                                              dt_miliseconds_{dt_miliseconds},
                                              interval_seconds_{interval_seconds}
  {
  }

  Path compute_trajectory(const Car &car,
                          const Path  &previous_path,
                          unsigned int target_lane,
                          double target_speed,
                          double target_accel,
                          bool changing_lanes);



 private:
  const Map &map_;
  double target_speed_meters_per_second_;
  double max_acceleration_ms2_;
  double min_distance_to_neighbor_;
  unsigned int dt_miliseconds_;
  unsigned int interval_seconds_;
  vector<double> prev_speed_;

};

#endif