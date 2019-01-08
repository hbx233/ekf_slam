#ifndef LASER_FRAME_H_
#define LASER_FRAME_H_

#include "ekf_slam/common.h"
#include "ekf_slam/line_segment.h"
namespace ekf_slam{
class LaserFrame{
public: 
  using Ptr = shared_ptr<LaserFrame>;
  unsigned long id_; //laser reading id 
  double time_stamp_; //laser reading frame time stamp
  vector<Vector2d> laser_points_; //laser reading, represented as point in laser frame 
  //Vector3d T_w_l_; //laser sensor's pose in world frame 
  vector<LineSegment> line_segments_; //line segment features in laser frame 
public:
  //LaserFrame(const unsigned long& id, const double& time_stamp=0, const vector<Vector2d>& laser_points=vector<Vector2d>(), const Vector3d& T_w_l = Vector3d());
  LaserFrame(const unsigned long& id, const double& time_stamp=0, const vector<Vector2d>& laser_points=vector<Vector2d>());
};
}
#endif
