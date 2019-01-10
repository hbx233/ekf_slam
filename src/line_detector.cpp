#include "ekf_slam/line_detector.h"
namespace ekf_slam{
  
LineDetector::LineDetector(const double& distance_threshold, const int& min_points_num)
  : distance_threshold_(distance_threshold),min_points_num_(min_points_num){}

LineDetector::Ptr LineDetector::create(const double& line_distance_threshold, const int& min_points_num)
{
  //return std::make_shared<LineDetector>(line_distance_threshold,min_points_num);
  //why cannot use make_shared? If use make_shared, it cannot access the private constructor 
  return shared_ptr<LineDetector>(new LineDetector(line_distance_threshold,min_points_num));
}

void LineDetector::detect(const vector< Vector2d >& points_2d, vector< LineSegment >& line_segments)
{
  cout<<"Begin to extract LineSegments"<<endl;
  //get original split start and end index 
  //int start_idx = 0;
  //int end_idx = laser_points_.size();
  //recursive split 
  vector<LineSegment> splitted_line_segs = splitLines(points_2d,0,points_2d.size());
  //Just for test:
  //vector<LineSegment> splitted_line_segs = splitLines(points_2d,0,150);
  //merge splitted line segments 
  vector<LineSegment> merged_line_segs = mergeColinearNeighbors(points_2d, splitted_line_segs);
  //filter out the short line segments
  for(int i=0; i<merged_line_segs.size(); i++){
    if((merged_line_segs[i].end_idx_-merged_line_segs[i].start_idx_)>=min_points_num_){
      line_segments.push_back(merged_line_segs[i]);
    }
  }
  cout<<"End Merging LineSegments"<<endl;
  cout<<"Extracted "<<line_segments.size()<<" lines"<<endl;
}
LineSegment LineDetector::fitLine(const vector< Vector2d >& points_2d, const int& start_idx, const int& end_idx)
{
  //find the reference point 
  //Note: end_idx refers to excluded end point
  Vector2d center_point = std::accumulate(points_2d.begin()+start_idx, points_2d.begin()+end_idx, Vector2d(0,0));
  center_point = center_point / (end_idx-start_idx);//exclude end point, include start point 
  //calculate alpha, angle of the polar representation of line 
  double num = 0;//numerator 
  double denom = 0; //denominator 
  for(int i=start_idx;i<end_idx;i++){
    num += (points_2d[i][0]-center_point[0]) * (points_2d[i][1]-center_point[1]);
    denom += (std::pow(points_2d[i][1]-center_point[1],2) - std::pow(points_2d[i][0]-center_point[0],2));
  }
  num *= -2;
  double alpha = 0.5 * atan2(num,denom);
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
int LineDetector::findSplitIndex(const vector<Vector2d>& points_2d,const LineSegment& line_segment)
{
  if((line_segment.end_idx_-line_segment.start_idx_)<=2){
    //only contains 2 points or lesser 
    return -1;
  }
  int split_idx;
  //compute the distance from point to line segment 
  vector<double> distance(line_segment.end_idx_-line_segment.start_idx_,0);
  vector<int> far_dist_idx;
  vector<double> far_away_distance;
  //compute the signed distance 
  for(int i=line_segment.start_idx_;i<line_segment.end_idx_;i++){
    distance[i-line_segment.start_idx_] = (points_2d[i][0] * cos(line_segment.alpha_) + points_2d[i][1] * sin(line_segment.alpha_) - line_segment.r_);
  }
  for(int i=0; i<distance.size()-1; i++){
    bool _flag0 = std::abs(distance[i])>=distance_threshold_;
    bool _flag1 = std::abs(distance[i+1])>=distance_threshold_;
    bool _flag2 = (distance[i] * distance[i+1]) > 0;
    if(_flag0 & _flag1 & _flag2){
      far_dist_idx.push_back(i);
      far_away_distance.push_back(std::abs(distance[i]+distance[i+1]));
    }
  }
  if(far_dist_idx.size()==0){
    return -1;
  }

  auto max_it = std::max_element(far_away_distance.begin(),far_away_distance.end());
  int _idx = far_dist_idx[max_it - far_away_distance.begin()];
  if(std::abs(distance[_idx])<std::abs(distance[_idx+1])){
    _idx++;
  }
  split_idx = _idx + line_segment.start_idx_;
  if(split_idx==line_segment.start_idx_){
    split_idx = line_segment.start_idx_+1;
  }
  if(split_idx==line_segment.end_idx_){
    split_idx = line_segment.end_idx_-1;
  }
  return split_idx;
}
vector< LineSegment > LineDetector::splitLines(const vector< Vector2d >& points_2d, const int& start_idx, const int& end_idx)
{
  vector<LineSegment> line_segs;
  //fit a line on current segment 
  LineSegment line_seg = fitLine(points_2d, start_idx, end_idx);
  //if lesser or equal to two points, no need to split
  if((end_idx-start_idx)<=2){
    line_segs.push_back(line_seg);
    return line_segs;
  }
  //find a split point 
  int split_idx = findSplitIndex(points_2d, line_seg);
  if(split_idx != -1){
    //split the current line segment
    vector<LineSegment> line_segs_half_1 = splitLines(points_2d,start_idx,split_idx);//split points from start_idx to split_idx
    vector<LineSegment> line_segs_half_2 = splitLines(points_2d,split_idx,end_idx);//split points from split_idx + 1 to end_idx-1
    //merge two line segments and return 
    //make them still stored as neighbor, which is necessary for merge process
    line_segs_half_1.insert(line_segs_half_1.end(),line_segs_half_2.begin(),line_segs_half_2.end());
    return line_segs_half_1;
  } else{
    //no need to split 
    line_segs.push_back(line_seg);
    return line_segs;
  }
}
vector< LineSegment > LineDetector::mergeColinearNeighbors(const vector<Vector2d>& points_2d, const vector< LineSegment >& line_segments)
{
  vector<LineSegment> merged_line_segments;
  //set the current line segment which is used to merge the following lines
  LineSegment curr_line_seg = line_segments[0];
  int curr_start_idx = curr_line_seg.start_idx_;
  int curr_end_idx = curr_line_seg.end_idx_;
  for(int i = 1; i<line_segments.size(); i++){
    if(curr_end_idx!=line_segments[i].start_idx_){
      ERROR_MSG("Not a neighbor");
      cout<<"Current LineSegment's end_idx:  "<<curr_end_idx;
      cout<<i<<"th LineSegment's start_idx:  "<<line_segments[i].start_idx_;
    }
    curr_end_idx = line_segments[i].end_idx_;
    //fit a line using points from both current line segment to be merged
    //and the neighbor line segment 
    LineSegment new_line_seg = fitLine(points_2d, curr_start_idx,curr_end_idx);
    //try to split the new line segment 
    int split_idx = findSplitIndex(points_2d,new_line_seg);
    if(split_idx==-1){
      //cannot be splitted, merge two line segments as the new_line_seg
      curr_line_seg = new_line_seg; 
    } else{
      //have split point, cannot be merged
      //push curr_line_seg to final line segments
      merged_line_segments.push_back(curr_line_seg);
      //update curr_line_seg and index
      curr_line_seg = line_segments[i];
      curr_start_idx = curr_line_seg.start_idx_;
      curr_end_idx = curr_line_seg.end_idx_;
    }
  }
  //add last line segment
  merged_line_segments.push_back(curr_line_seg);
  return merged_line_segments;
}
#if 0
//used only for debug
void LineDetector::display_line_segments(const vector<Vector2d>& points_2d,const vector<LineSegment>& line_segments)
{
  int w = 1000;
  cv::Mat img = cv::Mat(w,w,CV_8UC3,cv::Scalar(255,255,255));
  int color = 0;
  for(int i=0; i<line_segments.size(); i++){
    for(int j=line_segments[i].start_idx_;j<line_segments[i].end_idx_;j++){
      cv::Point pt(static_cast<int>(points_2d[j][1]*100+500),1000-static_cast<int>(points_2d[j][0]*100+500));
      switch(color){
	case 0: cv::circle(img,pt,1,cv::Scalar(0,0,0));break;
	case 1: cv::circle(img,pt,1,cv::Scalar(0,255,0));break;
	case 2: cv::circle(img,pt,1,cv::Scalar(255,0,0));break;
      }
    }
    if(color==2){
      color=0;
    } else{
      color++;
    }
#define DISPLAY_LINES 0
#if DISPLAY_LINES
    cv::Point pt1(static_cast<int>(points_2d[line_segments[i].start_idx_][1]*100+500),1000-static_cast<int>(points_2d[line_segments[i].start_idx_][0]*100+500));
    cv::Point pt2(static_cast<int>(points_2d[line_segments[i].end_idx_-1][1]*100+500),1000-static_cast<int>(points_2d[line_segments[i].end_idx_-1][0]*100+500));
    int x = 500 - line_segments[i].r_ * cos(line_segments[i].alpha_) * 100;
    int y = line_segments[i].r_ * sin(line_segments[i].alpha_) * 100 + 500;
    cv::Point pt_c(y,x);
    cv::circle(img,pt_c,1,cv::Scalar(255,0,0),2);
    cv::line(img,pt1,pt2,cv::Scalar(0,200,0));
#endif
  }
  cv::imshow("Laser Points",img);
  cv::waitKey(0);
}
#endif
}