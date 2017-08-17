/*************************************************************************
  > File Name: data_collection.h
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Di 15 Aug 2017 15:00:06 CEST
 ************************************************************************/

#ifndef _DATA_COLLECTION_H
#define _DATA_COLLECTION_H
#include <vector>
#include <thread>
#include <memory>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <data_collection/DataCollectionService.h>
#include <ros/ros.h>

namespace DC{

  template<typename topicType>
    class GetTopicVector{
      public: 
        std::vector<topicType> StateVector;
        GetTopicVector(
            const ros::NodeHandlePtr, 
            const std::string, 
            const int=5);
      private:
        int vector_size;
        ros::Subscriber StateSub;
        void StateCallback(const topicType&);
    };

  using FORCE_TYPE = geometry_msgs::Twist;
  using SCAN_TYPE  = sensor_msgs::LaserScan; 
  using RGB_IMAGE_TYPE  = sensor_msgs::ImageConstPtr; 
  using DEPTH_IMAGE_TYPE  = sensor_msgs::ImageConstPtr; 
  //const std::string FORCE_NAME = "/actor0/vel";
  const std::string SCAN_NAME =  "/scan";
  const std::string RGB_IMAGE_NAME = "/camera/rgb/image_raw"; 
  const std::string DEPTH_IMAGE_NAME = "/camera/depth/image_raw"; 

  class DataCollection{
    private:
      const ros::NodeHandlePtr rosNodeConstPtr;
      ros::ServiceServer dc_Service;
      std::shared_ptr<DC::GetTopicVector<DC::FORCE_TYPE>> force_sub_ptr;
      std::shared_ptr<DC::GetTopicVector<DC::SCAN_TYPE>> scan_sub_ptr;
      std::shared_ptr<DC::GetTopicVector<DC::RGB_IMAGE_TYPE>> rgb_sub_ptr;
      std::shared_ptr<DC::GetTopicVector<DC::DEPTH_IMAGE_TYPE>> depth_sub_ptr;
      
      const float sleeping_time;
      
      std::string actor_name;
    public:
      bool ServiceCallback(
          data_collection::DataCollectionService::Request&,
          data_collection::DataCollectionService::Response&);
      DataCollection(
          const std::string ="data_collection_service",
          const std::string ="data_collection",
          const float =0.01);
  };
}

#endif
