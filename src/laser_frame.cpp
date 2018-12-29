#include "ekf_slam/common.h"
#include "ekf_slam/laser_frame.h"
namespace ekf_slam{
LaserFrame::LaserFrame(const long unsigned int& id, const double& time_stamp, const vector<Vector2d>& laser_points, const SE2& T_w_l, const double& line_distance_threshold, const int& min_segment_length)
  :id_(id),time_stamp_(time_stamp),laser_points_(laser_points),T_w_l_(T_w_l),line_distance_threshold_(line_distance_threshold),min_segment_length_(min_segment_length)
{}


LineSegment LaserFrame::fitLine(const int& start_idx, const int& end_idx)
{
  //find the reference point 
  //Note: end_idx refers to excluded end point
  Vector2d center_point = std::accumulate(laser_points_.begin()+start_idx, laser_points_.begin()+end_idx, Vector2d(0,0));
  center_point = center_point / (end_idx-start_idx);//exclude end point, include start point 
  //calculate alpha, angle of the polar representation of line 
  double num = 0;//numerator 
  double denom = 0; //denominator 
  for(int i=start_idx;i<end_idx;i++){
    num += (laser_points_[i][0]-center_point[0]) * (laser_points_[i][1]-center_point[1]);
    denom += (std::pow(laser_points_[i][1]-center_point[1],2) - std::pow(laser_points_[i][0]-center_point[0],2));
  }
  num *= -2;
  double alpha = atan2(num,denom);
  //calculate r of polar representation of line 
  double r = cos(alpha) * center_point[0] + sin(alpha) * center_point[1];
  //eliminate negative radii 
  if(r<0){
    alpha += 3.1415926;
    if(alpha>3.1415926){
      alpha-=2*3.1415926;
    }
    r = -r;
  }
  return LineSegment(alpha,r,start_idx,end_idx);
}
int LaserFrame::findSplitIndex(const LineSegment& line_seg)
{
  //compute the distance from point to line segment 
  vector<double> distance(line_seg.end_idx_-line_seg.start_idx_,0);
  for(int i=line_seg.start_idx_;i<line_seg.end_idx_;i++){
    distance[i] = (laser_points_[i][0] * cos(line_seg.alpha_) + laser_points_[i][1] * sin(line_seg.alpha_) - line_seg.r_);
  }
  //compute the split point index from distance 
  //use a weighting system to weight every point in the line segment to be a split point 
  //1. greater than the distance threshold, if not, weight should be zero 
  //2. not a spur point, which means at least will have a neighbor with the 
  //   the same sign and greater than distance threshold, if not, weight should be zero 
  //3. two neighbor points' distance are all smaller than the central point
  vector<double> weight(line_seg.end_idx_-line_seg.start_idx_,0);
  //loop through the 1 - N-2 points(original: 0-N-1) 
  //both end of the segment cannot be the split point 
  for(int i=1;i<distance.size()-1;i++){
    bool _flag1 = std::abs(distance[i])>line_distance_threshold_;
    bool _flag2 = std::abs(distance[i-1])>line_distance_threshold_;
    bool _flag3 = std::abs(distance[i+1])>line_distance_threshold_;
    bool _flag4 = (distance[i] * distance[i-1])>0;
    bool _flag5 = (distance[i] * distance[i+1])>0;
    bool _flag6 = (std::abs(distance[i])>std::abs(distance[i-1]) || std::abs(distance[i]));
    //flag 1: greater than threshold
    //flag 2 && flag 4: neighbor i-1 is greater than threshold and on the same side with i 
    //flag 3 && flag 5: neighbor i+1 is greater than threshold and on the same side with i
    double w = 0;
    if(_flag1 && ((_flag2 && _flag4) || (_flag3 && _flag5))){
      w = 1;
      if(_flag2 && _flag4 && _flag3 && _flag5){
	//supported by two neighbors 
	w += 0.15;
      }
      if(_flag6){
	//both neighbors's distance smaller than the central one 
	w += 0.3;
      }
    }
    weight[i] = distance[i]*w;//compute weight with local weight and distance 
  }
  //find the point with maximun weight 
  auto max_it = std::max_element(weight.begin(),weight.end());
  if(*max_it<0.01){
    return -1;//no split point, the line segment cannot be splitted 
  } else{
    return max_it - weight.begin() + line_seg.start_idx_;//add the original start index 
  }
}
//recursive function to split all the lines 
vector<LineSegment> LaserFrame::splitLines(int start_idx, int end_idx)
{
  vector<LineSegment> line_segs;
  //fit a line on current segment 
  LineSegment line_seg = fitLine(start_idx, end_idx);
  //if lesser or equal to two points, no need to split
  if((end_idx-start_idx)<=2){
    line_segs.push_back(line_seg);
    return line_segs;
  }
  //find a split point 
  int split_idx = findSplitIndex(line_seg);
  if(split_idx != -1){
    //split the current line segment
    vector<LineSegment> line_segs_half_1 = splitLines(start_idx,split_idx+1);//split points from start_idx to split_idx
    vector<LineSegment> line_segs_half_2 = splitLines(split_idx+1,end_idx);//split points from split_idx + 1 to end_idx-1
    //merge two line segments and return 
    //make them still stored as neighbor, which is necessary for merge process
    line_segs_half_1.insert(line_segs_half_1.end(),line_segs_half_2.begin(),line_segs_half_2.end());
    return line_segs_half_1;
  } else{
    //no need to split 
    line_segs.push_back(line_seg);
    return;
  }
}
vector<LineSegment> LaserFrame::mergeColinearNeighbors(const vector<LineSegment>& line_segs)
{
  vector<LineSegment> line_segments;
  //set the current line segment which is used to merge the following lines
  LineSegment curr_line_seg = line_segs[0];
  int curr_start_idx = curr_line_seg.start_idx_;
  int curr_end_idx = curr_line_seg.end_idx_;
  for(int i = 1; i<line_segs.size(); i++){
    if(curr_end_idx!=line_segs[i].start_idx_){
      ERROR_MSG("Not a neighbor");
      cout<<"Current LineSegment's end_idx:  "<<curr_end_idx;
      cout<<i<<"th LineSegment's start_idx:  "<<line_segs[i].start_idx_;
    }
    curr_end_idx = line_segs[i].end_idx_;
    //fit a line using points from both current line segment to be merged
    //and the neighbor line segment 
    LineSegment new_line_seg = fitLine(curr_start_idx,curr_end_idx);
    //try to split the new line segment 
    int split_idx = findSplitIndex(new_line_seg);
    if(split_idx==-1){
      //cannot be splitted, merge two line segments as the new_line_seg
      curr_line_seg = new_line_seg; 
    } else{
      //have split point, cannot be merged
      //push curr_line_seg to final line segments
      line_segments.push_back(curr_line_seg);
      //update curr_line_seg and index
      curr_line_seg = line_segs[i];
      curr_start_idx = curr_line_seg.start_idx_;
      curr_end_idx = curr_line_seg.end_idx_;
    }
  }
  //add last line segment
  line_segments.push_back(curr_line_seg);
}

void LaserFrame::extractLineSegments()
{
  //get original split start and end index 
  int start_idx = 0;
  int end_idx = laser_points_.size();
  //recursive split 
  vector<LineSegment> splitted_line_segs = splitLines(start_idx,end_idx);
  //merge splitted line segments 
  vector<LineSegment> merged_line_segs = mergeColinearNeighbors(splitted_line_segs);
  //filter out the short line segments
  for(int i=0; i<merged_line_segs.size(); i++){
    if((merged_line_segs[i].end_idx_-merged_line_segs[i].start_idx_)>=min_segment_length_){
      line_segments_.push_back(merged_line_segs[i]);
    }
  }
}

}

