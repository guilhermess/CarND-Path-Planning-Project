#ifndef NEIGHBOR_CARS_HPP
#define NEIGHBOR_CARS_HPP

#include <vector>
#include <cmath>
#include <iostream>

#include "Map.hpp"

using std::vector;
using std::pair;

class NeighborCar
{
 public:
  NeighborCar(int id,
              double x,
              double y,
              double vx,
              double vy,
              double s,
              double d) : id_{id},
                          x_{x}, y_{y},
                          vx_{vx}, vy_{vy},
                          s_{s}, d_{d}
  {}

  int id() const { return id_;}
  double x() const {return x_;}
  double y() const {return y_;}
  double vx() const {return vx_;}
  double vy() const {return vy_;}
  double s() const {return s_;}
  double d() const {return d_;}

  unsigned int lane() const
  {
    return static_cast<int>(std::floor(d_/Map::LANE_WIDTH));
  }

  pair<double,double> estimate_car_frenet_position(unsigned int interval_seconds) const;

 private:
  int id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double s_;
  double d_;
};

class NeighborCars
{
 public:
  inline NeighborCars(const vector<vector<double>> &cars,
                      int num_lanes) : lane_neighbors_(num_lanes)
  {
    for ( auto car : cars )
    {
      int id = static_cast<int>(car[0]);
      double x = car[1];
      double y = car[2];
      double vx = car[3];
      double vy = car[4];
      double s = car[5];
      double d = car[6];
      if ( d >= 0 && d < num_lanes * Map::LANE_WIDTH )
      {
        NeighborCar neighbor{id, x, y, vx, vy, s, d};
        lane_neighbors_[neighbor.lane()].push_back(neighbor);
      }
    }
  }

  inline const vector<NeighborCar> &get_neighbor_cars(unsigned int lane) const
  {
    return lane_neighbors_[lane];
  }

 private:
  vector<vector<NeighborCar>> lane_neighbors_;
};

#endif