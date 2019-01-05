#include "ekf_slam/line_segment.h"
namespace ekf_slam {
void LineSegment::convertToFrameT(const double& x, const double& y, const double& theta, LineSegment& ls_in_T, bool& isNegative) const
{
  ls_in_T.alpha_ = alpha_ - theta;
  ls_in_T.r_ = r_ - x * cos(alpha_) - y * sin(alpha_);
  if(ls_in_T.r_<0){
    ls_in_T.r_ = -ls_in_T.r_;
    ls_in_T.alpha_  += PI;
    isNegative = true;
  } else{
    isNegative = false;
  }
  if(ls_in_T.alpha_>PI){
    ls_in_T.alpha_ -= 2*PI;
  }
  if(ls_in_T.alpha_<-PI){
    ls_in_T.alpha_ += 2*PI;
  }
}
Vector2d LineSegment::vector() const
{
  return Vector2d(alpha_,r_);
}


}