#ifndef BEHAVIORAL_PLANNER_HPP
#define BEHAVIORAL_PLANNER_HPP

#include "Map.hpp"
#include "Predictor.hpp"
#include "NeighborCars.hpp"
#include "Path.hpp"
#include "Car.hpp"
#include "TrajectoryGenerator.hpp"
#include "SpeedEstimation.hpp"
#include <vector>

using std::vector;
using std::pair;


class BehavioralPlanner
{
 public:
  BehavioralPlanner(const Map &map,
                    TrajectoryGenerator &trajectory_generator,
                    SpeedEstimation &speed_estimator,
                    double max_acceleration_ms2,
                    double target_speed_ms) : map_{map},
                                              trajectory_generator_{trajectory_generator},
                                              speed_estimator_{speed_estimator},
                                              current_lane_{0},
                                              current_acceleration_{max_acceleration_ms2},
                                              target_speed_ms_{target_speed_ms},
                                              target_lane_{0}

  {}

  inline void set_current_lane(unsigned int lane)
  {
    current_lane_ = lane;
    target_lane_ = lane;
  }

  Path find_best_trajectory( const Car &car,
                             const vector<vector<NeighborPrediction>> &neighbor_estimated_positions,
                             const Path &previous_path);

 private:
  const Map &map_;
  TrajectoryGenerator &trajectory_generator_;
  SpeedEstimation &speed_estimator_;
  int current_lane_;
  double current_acceleration_;
  double target_speed_ms_;
  int target_lane_;

};

#endif