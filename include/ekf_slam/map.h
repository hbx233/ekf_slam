#ifndef MAP_H_
#define MAP_H_

#include "ekf_slam/line_segment.h"
#include "ekf_slam/line_detector.h"
namespace ekf_slam{
class Map{
public:
  using Ptr = shared_ptr<Map>;
  Map():line_segments_(vector<LineSegment>()),P_line_segments_(vector<Matrix2d>()) {};
  Map(const vector<LineSegment>& line_segments, const vector<Matrix2d>& P_line_segments);
public:
  void addOneLineSegment(const LineSegment& line_segment, const Matrix2d& P_line_segment);
  void updateExistingLineSegments(const vector<LineSegment>& line_segments, const vector<Matrix2d>& P_line_segments);
  size_t size() const;
  const vector<LineSegment>& getMap() const;
  MatrixXd getCovarianceMatrix() const;
private:
  
  vector<LineSegment> line_segments_;
  vector<Matrix2d> P_line_segments_;
};
}
#endif