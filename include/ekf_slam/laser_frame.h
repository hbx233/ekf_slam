#ifndef FRAME_H_
#define FRAME_H_

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
  Line(const double& alpha, const double& r, const int& start_idx, const int& end_idx)
    :alpha_(alpha),r_(r),start_idx_(start_idx),end_idx_(end_idx){}
};
class LaserFrame{
public: 
  using Ptr = unique_ptr<LaserFrame>;
  unsigned long id_; //laser reading id 
  double time_stamp_; //laser reading frame time stamp
  vector<Vector2d> laser_points_; //laser reading, represented as point in laser coordinate 
  SE2 T_w_l_; //laser sensor's pose in world frame 
  vector<LineSegment> line_segments_;
  double line_distance_threshold_{0.1};
  int min_segment_length_{3};
public:
  LaserFrame(){};
  LaserFrame(const unsigned long& id, const double& time_stamp=0, const vector<Vector2d>& laser_points=vector<Vector2d>(), const SE2& T_w_l=SE2(), 
	     const double& line_distance_threshold, const int& min_segment_length);
  void extractLineSegments();
private:
  /**
   * internal methed for line extraction 
   */
  LineSegment fitLine(const int& start_idx, const int& end_idx);
  vector<LineSegment> splitLines(const int& start_idx, const int& end_idx);
  int findSplitIndex(const LineSegment& line_seg);
  vector<LineSegment> mergeColinearNeighbors(const vector<LineSegment> line_segs);
};
}
#endif
