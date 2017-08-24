#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "Map.hpp"
#include "BehavioralPlanner.hpp"
#include "Predictor.hpp"
#include "TrajectoryGenerator.hpp"
#include "SpeedEstimation.hpp"
#include "Car.hpp"
#include "NeighborCars.hpp"
#include "Path.hpp"

class PathPlanner
{
 public:
  PathPlanner(const string &map_file,
              int num_lanes,
              double target_speed_mph,
              double max_acceleration_meters_per_second,
              double min_distance_to_neighbor,
              unsigned int dt_ms,
              unsigned int interval_seconds ) : map_{map_file, num_lanes},
                                                target_speed_meters_per_second_{target_speed_mph*0.44704},
                                                max_acceleration_meters_per_second_{max_acceleration_meters_per_second},
                                                min_distance_to_neighbor_{min_distance_to_neighbor},
                                                dt_ms_{dt_ms},
                                                interval_seconds_{interval_seconds},
                                                predictor_{map_, interval_seconds_, num_lanes},
                                                trajectory_generator_{map_,
                                                                      target_speed_meters_per_second_,
                                                                      max_acceleration_meters_per_second_,
                                                                      min_distance_to_neighbor_,
                                                                      dt_ms_,
                                                                      interval_seconds},
                                                speed_estimator_{dt_ms_, interval_seconds_,
                                                                 max_acceleration_meters_per_second_,
                                                                 target_speed_meters_per_second_},
                                                behavioral_planner_{map_, trajectory_generator_, speed_estimator_,
                                                                    max_acceleration_meters_per_second_,
                                                                    target_speed_meters_per_second_}
  {}

  inline void set_current_lane(unsigned int lane)
  {
    behavioral_planner_.set_current_lane(lane);
  }

  Path calculate_path(const Car &car,
                      const NeighborCars &neighbors,
                      const Path &previous_path);

  inline const Map &map() const { return map_;}

 private:
  Map map_;
  double target_speed_meters_per_second_;
  double max_acceleration_meters_per_second_;
  double min_distance_to_neighbor_;
  unsigned int dt_ms_;
  unsigned int interval_seconds_;
  Predictor predictor_;
  TrajectoryGenerator trajectory_generator_;
  SpeedEstimation speed_estimator_;
  BehavioralPlanner behavioral_planner_;

};

#endif