#include "rnav/local_planner//trajectory.h"
#include <assert.h>
#include <iostream>
using namespace std;
namespace tri_local_planner {
  Trajectory::Trajectory()
    : xv_(0.0), yv_(0.0), thetav_(0.0), cost_(-1.0)
  {
  }

  Trajectory::Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts)
    : xv_(xv), yv_(yv), thetav_(thetav), cost_(-1.0), time_delta_(time_delta), x_pts_(num_pts), y_pts_(num_pts), th_pts_(num_pts)
  {
  }

  void Trajectory::getPoint(unsigned int index, double& x, double& y, double& th) const {
  if(index >= x_pts_.size()) cout<<"index x: "<<index<<" size x: "<<x_pts_.size()<<endl;
  if(index >= y_pts_.size()) cout<<"index y: "<<index<<" size y: "<<y_pts_.size()<<endl;
  if(index >= th_pts_.size()) cout<<"index th: "<<index<<" size th: "<<th_pts_.size()<<endl;
  assert(index < x_pts_.size()); 
  assert(index < y_pts_.size());
  assert(index < th_pts_.size());
    x = x_pts_[index];
    y = y_pts_[index];
    th = th_pts_[index];
  }

  void Trajectory::setPoint(unsigned int index, double x, double y, double th){
    x_pts_[index] = x;
    y_pts_[index] = y;
    th_pts_[index] = th;
  }

  void Trajectory::addPoint(double x, double y, double th){
    x_pts_.push_back(x);
    y_pts_.push_back(y);
    th_pts_.push_back(th);
  }

  void Trajectory::resetPoints(){
    x_pts_.clear();
    y_pts_.clear();
    th_pts_.clear();
  }

  void Trajectory::getEndpoint(double& x, double& y, double& th) const {
    x = x_pts_.back();
    y = y_pts_.back();
    th = th_pts_.back();
  }

  unsigned int Trajectory::getPointsSize() const {
    return x_pts_.size();
  }
};
    
