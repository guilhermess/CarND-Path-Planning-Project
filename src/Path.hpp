
#ifndef PATH_HPP
#define PATH_HPP

class Path
{
 public:
  Path(const vector<double> &x,
       const vector<double> &y,
       double end_s=-1, double end_d=-1) :
    x_{x}, y_{y}, end_s_{end_s}, end_d_{end_d}
  {}

  const vector<double> &get_x() const { return x_;}
  const vector<double> &get_y() const { return y_;}

  double get_end_s() const {return end_s_;}
  double get_end_d() const {return end_d_;}

  inline std::size_t size() const { return x_.size();}

 private:
  vector<double> x_;
  vector<double> y_;

  double end_s_;
  double end_d_;

};


#endif