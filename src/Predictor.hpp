
#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include "Map.hpp"
#include "NeighborCars.hpp"
#include <vector>

using std::vector;
using std::pair;

struct NeighborPrediction
{
  NeighborPrediction(double curr_s, double curr_d,
                     double est_s, double est_d) :
  current_s{curr_s},
  current_d{curr_d},
  estimated_s{est_s},
  estimated_d{est_d}
  {}

  double current_s;
  double current_d;
  double estimated_s;
  double estimated_d;


};

class Predictor
{
 public:
  Predictor(const Map &map,
            unsigned int interval_seconds,
            int num_lanes) : map_{map},
                                      interval_seconds_{interval_seconds},
                                      num_lanes_{num_lanes}
  {}

  vector<vector<NeighborPrediction>> predict_neighbor_positions(const NeighborCars &neighbors) const;

 private:
  const Map &map_;
  unsigned int interval_seconds_;
  int num_lanes_;

};

#endif