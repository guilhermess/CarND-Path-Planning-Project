
#include "PathPlanner.hpp"


Path PathPlanner::calculate_path(const Car &car,
                                 const NeighborCars &neighbors,
                                 const Path &previous_path)
{
  auto neighbor_estimated_positions = predictor_.predict_neighbor_positions(neighbors);
  return behavioral_planner_.find_best_trajectory(car, neighbor_estimated_positions, previous_path);
}
