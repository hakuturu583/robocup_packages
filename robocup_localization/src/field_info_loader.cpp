#include <field_info_loader.h>
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robocup_msgs/RobotLocation.h>
#include <math.h>
#include <ros/ros.h>

field_info_loader::field_info_loader(std::string file_name)
{
  field_height = 9.05;
  field_width = 6.05;
  std::string package_path = ros::package::getPath("robocup_localization");
  file_path = package_path + "/data/" + file_name;
  field_info = cv::imread(file_path,0);
  cv::threshold(field_info,field_info,100,255,cv::THRESH_BINARY);
}

//if pointed value is out of area return -1
//if pointed value is on the white line returns 1
//if pointed value is on the green field returns 0
int field_info_loader::point_type(double x,double y)
{
  if(fabs(y) > field_height/2 || fabs(x) > field_width/2)
  {
    return -1;
  }
  double x_cv = (field_height/2)- y;
  double y_cv = (field_width/2) + x;
  int x_index = (int)(x_cv/field_height*field_info.cols);
  int y_index = (int)(y_cv/field_width*field_info.rows);
  if(x_index < 0 || x_index >= field_info.cols)
  {
    return -1;
  }
  if(y_index < 0 || y_index >= field_info.rows)
  {
    return -1;
  }
  int cell_value = field_info.data[y_index*field_info.step+x_index*field_info.elemSize()+field_info.channels()];
  if(cell_value == 0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}
