
#include "Predictor.hpp"


vector<vector<NeighborPrediction>> Predictor::predict_neighbor_positions(const NeighborCars &neighbors) const
{
  vector<vector<NeighborPrediction>> neighbor_positions(num_lanes_);
  for ( unsigned int i = 0; i < num_lanes_; ++i)
    for ( auto neighbor : neighbors.get_neighbor_cars(i)) {
      auto estimated_sd = neighbor.estimate_car_frenet_position(interval_seconds_);
      neighbor_positions[i].push_back(NeighborPrediction( neighbor.s(), neighbor.d(),
                                                          estimated_sd.first, estimated_sd.second));
    }

  return neighbor_positions;
}