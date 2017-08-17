/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:45:03 CEST
 ************************************************************************/

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "data_collection.h"

int main(int argc, char **argv){
  
  std::string node_name = "data_collection";
  ros::init(argc, argv, node_name);
  
  DC::DataCollection datacollection;
  
  //ros::spin();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}

