
#include "BehavioralPlanner.hpp"
#include <iostream>
#include <limits>

using namespace std;

Path BehavioralPlanner::find_best_trajectory( const Car &car,
                                              const vector<vector<NeighborPrediction>> &neighbor_estimated_positions,
                                              const Path &previous_path)
{
  vector<int> delta_lane_options = {0, 1, -1};

  std::cout << "BEGIN BehaviorPlanner current_lane: " << current_lane_ << " target: " << target_lane_ << std::endl;

  int best_lane = current_lane_;
  double best_lane_cost = std::numeric_limits<double>::max();
  double best_speed;
  double best_accel;

  vector<pair<double,double>> lane_speed_accel(3, make_pair(0,0));

  if ( target_lane_ != current_lane_ )
  {
    if ( fabs(Map::get_d(target_lane_) - car.d) < 1.0 )
      current_lane_ = target_lane_;
  }

  for ( auto delta_lane : delta_lane_options ) {
    unsigned int lane = current_lane_ + delta_lane;
    if ( lane < 0 || lane >= map_.get_num_lanes() )
      continue;

    auto lane_neighbors = neighbor_estimated_positions[lane];

    double min_distance = std::numeric_limits<double>::max();
    for ( auto neighbor : lane_neighbors ) {
      double distance = neighbor.estimated_s;
      double car_s = car.s;
      if (previous_path.size() != 0)
        car_s = min(car.s, previous_path.get_end_s());

      double delta_distance = distance - car_s;

      if ( ( (lane == current_lane_ && neighbor.current_s > car.s ) ||
             (lane != current_lane_ && ( (neighbor.current_s - car.s) >= -10 )) )
           & delta_distance < min_distance ) {
        min_distance = delta_distance;
      }
    }

    pair<double,double> speed_accel;
    if ( min_distance == std::numeric_limits<double>::max() || min_distance < -25.0 ) {
      speed_accel = speed_estimator_.estimate_speed_acceleration(90, car.speed, 30);
    }
    else {
      speed_accel = speed_estimator_.estimate_speed_acceleration(min_distance, car.speed, 30);
    }

    lane_speed_accel[lane] = speed_accel;
    double lane_cost = 1.0 * std::abs(delta_lane) + 1.0 * (target_speed_ms_ - speed_accel.first);

    if ( lane_cost < best_lane_cost )
    {
      best_lane_cost = lane_cost;
      best_lane = lane;
      best_speed = speed_accel.first;
      best_accel = speed_accel.second;
    }

    std::cout << "\tlane : " << lane << " cost: " << lane_cost << " speed: " << speed_accel.first << " min_distance: " << min_distance << std::endl;
  }

  bool changing_lane = false;
  if ( target_lane_ != current_lane_ )
  {
    changing_lane = true;
    if ( fabs(Map::get_d(target_lane_) - car.d) > 1.0 )
    {
      auto speed_accel = lane_speed_accel[Map::get_lane(car.d)];
      best_speed = speed_accel.first;
      best_accel = speed_accel.second;
    }
  }
  else
  {
    target_lane_ = best_lane;
  }

  std::cout << "\tbest_lane: " << best_lane << std::endl;

  return trajectory_generator_.compute_trajectory(car,
                                                  previous_path,
                                                  target_lane_,
                                                  best_speed,
                                                  best_accel,
                                                  changing_lane);

}
