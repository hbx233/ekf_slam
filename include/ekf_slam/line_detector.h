#ifndef LINE_DETECTOR_H_
#define LINE_DETECTOR_H_
#include "ekf_slam/common.h"
#include "ekf_slam/laser_frame.h"
namespace ekf_slam{

class LineDetector{
public:
  using Ptr = shared_ptr<LineDetector>;
public:
  //
  static Ptr create(const double& distance_threshold=0.01, const int& min_points_num=20);
  //detect line segments from given 2d points(double) 
  void detect(const vector<Vector2d>& points_2d, vector<LineSegment>& line_segments);
  //display the line segments 
  //void display_line_segments(const vector<Vector2d>& points_2d, const vector<LineSegment>& line_segments);
private:
  LineDetector(const double& distance_threshold, const int& min_points_num);
private:
  //private parameter field 
  double distance_threshold_;
  int min_points_num_;
private:
  LineSegment fitLine(const vector<Vector2d>& points_2d, const int& start_idx, const int& end_idx);
  vector<LineSegment> splitLines(const vector<Vector2d>& points_2d, const int& start_idx, const int& end_idx);
  int findSplitIndex(const vector<Vector2d>& points_2d,const LineSegment& line_segment);
  vector<LineSegment> mergeColinearNeighbors(const vector<Vector2d>& points_2d,const vector<LineSegment>& line_segments);
};


}
#endif