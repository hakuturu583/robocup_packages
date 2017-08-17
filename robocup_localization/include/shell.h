#ifndef SHELL_H_INCLUDED
#define SHELL_H_INCLUDED

//headers for boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//headers for ROS
#include <ros/ros.h>
#include <ros/console.h>

namespace shell
{
  void call(std::string command);
  void call_with_flag(std::string command, bool* flag);
  void call_and_fork(std::string command);
  void call_and_fork_with_flag(std::string command, bool* flag);
}

void shell::call(std::string command)
{
  try
  {
    if(ros::ok())
    {
      system(command.c_str());
    }
  }
  catch(std::exception e)
  {
    ROS_ERROR_STREAM(e.what() << " while calling:" << command);
  }
}

void shell::call_with_flag(std::string command, bool* flag)
{
  try
  {
    if(ros::ok())
    {
      system(command.c_str());
      *flag = true;
    }
  }
  catch(std::exception e)
  {
    ROS_ERROR_STREAM(e.what() << " while calling:" << command);
    *flag = false;
  }
}

void shell::call_and_fork(std::string command)
{
  boost::thread thread(boost::bind(&call, command));
}

void shell::call_and_fork_with_flag(std::string command, bool* flag)
{
  boost::thread thread(boost::bind(&call_with_flag, command, flag));
}

#endif
