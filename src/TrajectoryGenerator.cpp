
#include "TrajectoryGenerator.hpp"
#include <cmath>
#include <iostream>

#include "spline.h"

using namespace std;

Path TrajectoryGenerator::compute_trajectory(const Car &car,
                                             const Path  &previous_path,
                                             unsigned int target_lane,
                                             double target_speed,
                                             double target_accel,
                                             bool changing_lanes)
{
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = car.yaw;
  
  double prev_car_x;
  double prev_car_y;
  
  vector<double> points_x;
  vector<double> points_y;

  int prev_path_reuse = previous_path.size();
  if ( target_accel < 0 || changing_lanes ) {
    prev_path_reuse = min(10,prev_path_reuse);
  }

  if ( previous_path.size() < 2 )
  {
    prev_car_x = car.x - cos(car.yaw);
    prev_car_y = car.y - sin(car.yaw);
    
    points_x.push_back(prev_car_x);
    points_x.push_back(car.x);

    points_y.push_back(prev_car_y);
    points_y.push_back(car.y);
  }
  else
  {
    ref_x = previous_path.get_x()[prev_path_reuse - 1];
    ref_y = previous_path.get_y()[prev_path_reuse - 1];

    double ref_x_prev = previous_path.get_x()[prev_path_reuse - 2];
    double ref_y_prev = previous_path.get_y()[prev_path_reuse - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    points_x.push_back(ref_x_prev);
    points_x.push_back(ref_x);

    points_y.push_back(ref_y_prev);
    points_y.push_back(ref_y);
  }

  vector<double> displacement_points{ 30, 50, 70};
  double target_d = Map::get_d(target_lane);
  for ( auto s : displacement_points )
  {
    auto xy = map_.get_xy(car.s + s, target_d);

    points_x.push_back(xy.first);
    points_y.push_back(xy.second);
  }

  for ( int i = 0; i < points_x.size(); ++i )
  {
    auto shift_x = points_x[i] - ref_x;
    auto shift_y = points_y[i] - ref_y;

    points_x[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    points_y[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));

  }

  tk::spline spl;
  spl.set_points(points_x, points_y);

  vector<double> next_x;
  vector<double> next_y;


  for ( int i = 0; i < prev_path_reuse; ++i)
  {
    next_x.push_back(previous_path.get_x()[i]);
    next_y.push_back(previous_path.get_y()[i]);
  }

  double x_add_on = 0;
  int num_steps = interval_seconds_ * 1000 / dt_miliseconds_;
  unsigned int offset = num_steps - previous_path.size();
  double speed_meters_per_second = 0;
  if ( prev_speed_.size() > previous_path.size() ) {
    speed_meters_per_second = prev_speed_[prev_path_reuse];
  }
  vector<double> prev_speed;
  for ( int i = 0; i < prev_path_reuse; ++i )
    prev_speed.push_back(prev_speed_[offset+i]);

  for ( int i = 0; i < num_steps - prev_path_reuse; ++i)
  {
    speed_meters_per_second += target_accel/ num_steps;
    if ( target_accel > 0 )
      speed_meters_per_second = std::min(speed_meters_per_second, target_speed_meters_per_second_);
    else
      speed_meters_per_second = std::max(speed_meters_per_second, target_speed);

    speed_meters_per_second = std::max(speed_meters_per_second, 0.0);
    prev_speed.push_back(speed_meters_per_second);
    double x_point = x_add_on + (speed_meters_per_second * dt_miliseconds_ * 1e-3);
    double y_point = spl(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point +=ref_y;

    next_x.push_back(x_point);
    next_y.push_back(y_point);
  }
  prev_speed_ = prev_speed;

  return Path(next_x, next_y);
}

