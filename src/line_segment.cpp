#include "ekf_slam/line_segment.h"
namespace ekf_slam {
void LineSegment::convertToFrameT(const Vector3d& T)
{
  bool isNegative;
  Vector2d line = vectorInFrameT(T,isNegative);
  alpha_ = line(0);
  r_ = line(0);
}
Vector2d LineSegment::vector() const
{
  return Vector2d(alpha_,r_);
}

Vector2d LineSegment::vectorInFrameT(const Vector3d& T, bool& isNegative) const
{
  cout<<"*"<<endl;
  Vector2d line;
  line(0) = alpha_ - T(2);
  line(1) = r_ - T(0)*cos(alpha_) - T(1)*sin(alpha_);
  cout<<"**"<<endl;
  if(line(1)<0){
    line(1) = -line(1);
    line(0) = line(0) + PI;
    isNegative = true;
  } else{
    isNegative = false;
  }
  cout<<"***"<<endl;
  if(line(0)>PI){
    line(0) = line(0) - 2*PI;
  } 
  if(line(0)<-PI){
    line(0) = line(0) + 2*PI;
  }
  return line;
}

}