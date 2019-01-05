#include "ekf_slam/common.h"
#include "ekf_slam/laser_frame.h"

namespace ekf_slam{
LaserFrame::LaserFrame(const long unsigned int& id, const double& time_stamp, const vector< Vector2d >& laser_points, const Vector3d& T_w_l)
: id_(id),time_stamp_(time_stamp),laser_points_(laser_points),T_w_l_(T_w_l) {}


}

