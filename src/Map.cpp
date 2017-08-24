
#include "Map.hpp"
#include <fstream>
#include <sstream>
#include <limits>
#include <math.h>

using namespace std;

double Map::LANE_WIDTH = 4.0;

Map::Map(const string &mapFile, int num_lanes) : num_lanes_{num_lanes}
{
  ifstream in_map(mapFile.c_str(), ifstream::in);

  string line;
  while (getline(in_map, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    x_.push_back(x);
    y_.push_back(y);
    s_.push_back(s);
    dx_.push_back(d_x);
    dy_.push_back(d_y);
  }
}


int Map::ClosestWaypoint(double x, double y) const
{
  double closestLen = std::numeric_limits<double>::max();
  int closestWaypoint = 0;

  for(int i = 0; i < x_.size(); i++)
  {
    double map_x = x_[i];
    double map_y = y_[i];
    double dist = distance(x,y, map_x, map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}


int Map::NextWaypoint(double x, double y, double theta) const
{
  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = x_[closestWaypoint];
  double map_y = y_[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}


pair<double, double> Map::get_frenet(double x, double y, double theta) const
{
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = x_.size()-1;
  }

  double n_x = x_[next_wp]- x_[prev_wp];
  double n_y = y_[next_wp]- y_[prev_wp];
  double x_x = x - x_[prev_wp];
  double x_y = y - y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - x_[prev_wp];
  double center_y = 2000 - y_[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(x_[i],y_[i],x_[i+1],y_[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return make_pair(frenet_s,frenet_d);
}


pair<double, double> Map::get_xy(double s, double d) const
{
  int prev_wp = -1;

  while(s > s_[prev_wp+1] && (prev_wp < static_cast<int>(s_.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1) % x_.size();

  double heading = atan2((y_[wp2] - y_[prev_wp]),(x_[wp2] - x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - s_[prev_wp]);

  double seg_x = x_[prev_wp]+seg_s*cos(heading);
  double seg_y = y_[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return make_pair(x,y);

}
