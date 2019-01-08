#include "ekf_slam/map.h"

namespace ekf_slam {
Map::Map(const vector< LineSegment >& line_segments, const vector< Matrix2d >& P_line_segments)
{
  //check if the number of lines is equal to number of covariance matrix
  if(line_segments.size() == P_line_segments.size()){
    line_segments_ = line_segments;
    P_line_segments_ = P_line_segments;
  } else{
    ERROR_MSG("Numbers of LineSegments and Covariance Matrix are not the same");
  }
}
void Map::addOneLineSegment(const LineSegment& line_segment, const Matrix2d& P_line_segment)
{
  line_segments_.push_back(line_segment);
  P_line_segments_.push_back(P_line_segment);
}

void Map::updateExistingLineSegments(const vector< LineSegment >& line_segments, const vector<Matrix2d>& P_line_segments)
{
  if(line_segments.size() == line_segments_.size() && line_segments.size() == P_line_segments.size()){
    for(int i=0; i<line_segments.size(); i++){
      line_segments_[i].alpha_ = line_segments[i].alpha_;
      line_segments_[i].r_ = line_segments[i].r_;
      //can add filter functionality in future
      P_line_segments_[i] = P_line_segments[i];
    }
  }
}


MatrixXd Map::getCovarianceMatrix() const
{
  int n = P_line_segments_.size();
  MatrixXd P = MatrixXd::Zero(2*n,2*n);
  //make a block diagonal matrix 
  for(int i=0; i<n; i++){
    P.block(2*i,2*i,2,2) = P_line_segments_[i];
  }
  return P;
}

const vector< LineSegment >& Map::getMap() const
{
  return line_segments_;
}

size_t Map::size() const
{
  if(line_segments_.size() == P_line_segments_.size()){
    return line_segments_.size();
  } else{
    ERROR_MSG("Numbers of LineSegments and Covariance Matrix are not the same");
    return -1;
  }
}
}
