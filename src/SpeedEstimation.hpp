
#ifndef SPEED_ESTIMATION_HPP
#define SPEED_ESTIMATION_HPP

#include <utility>

using std::pair;

class SpeedEstimation
{
public:
  inline SpeedEstimation( unsigned int dt_miliseconds,
                          unsigned int interval_seconds,
                          double max_acceleration_ms2,
                          double target_speed_meters_per_second) :
      dt_miliseconds_{dt_miliseconds},
      interval_seconds_{interval_seconds},
      max_acceleration_ms2_{max_acceleration_ms2},
      target_speed_meters_per_second_{target_speed_meters_per_second}
  {}

  pair<double,double> estimate_speed_acceleration(double distance_to_car,
                                                  double current_speed_mps,
                                                  double target_distance_to_car) const;

private:
  unsigned int dt_miliseconds_;
  unsigned int interval_seconds_;
  double max_acceleration_ms2_;
  double target_speed_meters_per_second_;


};

#endif