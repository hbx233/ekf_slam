#ifndef LINE_SEGMENT_H_
#define LINE_SEGMENT_H_

#include "ekf_slam/common.h"
namespace ekf_slam{
/**
 * line structure using polar representation
 * r = x * cos(alpha) + y * sin(alpha)
 * End point is two points that 
 */
struct LineSegment{
  double alpha_;
  double r_;
  int start_idx_;
  int end_idx_;
  LineSegment(const double& alpha=0, const double& r=0, const int& start_idx=-1, const int& end_idx=-1)
    :alpha_(alpha),r_(r),start_idx_(start_idx),end_idx_(end_idx){}
  void convertToFrameT(const double& x, const double& y, const double& theta, LineSegment& ls_in_T, bool& isNegative) const;
  Vector2d vector() const;
  
};
}
#endif