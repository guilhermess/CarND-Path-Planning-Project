#ifndef MAP_HPP
#define MAP_HPP

#include <string>
#include <vector>
#include <cmath>

using std::vector;
using std::string;
using std::sqrt;
using std::pair;

class Map
{
 public:
  Map(const string &mapFile, int num_lanes);

  inline int get_num_lanes() const { return num_lanes_;}

  pair<double,double> get_frenet(double x, double y, double theta) const;

  pair<double,double> get_xy(double s, double d) const;

  static double LANE_WIDTH;

  static unsigned int get_lane(double d)
  {
    return static_cast<int>(std::floor(d/LANE_WIDTH));
  }

  static double get_d(double lane)
  {
    return LANE_WIDTH * (lane + 0.5);
  }

private:
  int num_lanes_;
  vector<double> x_;
  vector<double> y_;
  vector<float> s_;
  vector<float> dx_;
  vector<float> dy_;

  inline double distance(double x1, double y1, double x2, double y2) const
  {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  }

  int ClosestWaypoint(double x, double y) const;

  int NextWaypoint(double x, double y, double theta) const;

  double pi() const { return M_PI; }

};

#endif