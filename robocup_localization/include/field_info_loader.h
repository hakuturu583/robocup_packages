#ifndef FIELD_INFO_LOADER_H_INCLUDED
#define FIELD_INFO_LOADER_H_INCLUDED

#include <opencv/cv.h>
#include <robocup_msgs/RobotLocation.h>

class field_info_loader
{
public:
  field_info_loader(std::string file_name);
  int point_type(double x,double y);
private:
  cv::Mat field_info;
  double field_height;
  double field_width;
  std::string file_path;
};
#endif
