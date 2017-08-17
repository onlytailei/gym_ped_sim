/*************************************************************************
  > File Name: data_collection.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Di 15 Aug 2017 16:34:11 CEST
 ************************************************************************/

#include <iostream>
#include <mutex>
#include "data_collection.h"
using namespace std;


std::mutex topic_mutex;
template<typename topicType>
void DC::GetTopicVector<topicType>::StateCallback(const topicType& msg_){
  std::lock_guard<std::mutex> lock(topic_mutex);
  StateVector.push_back(msg_);
  if (StateVector.size() > vector_size)
    StateVector.erase(StateVector.begin());
}
////////////////////////////

template<typename topicType>
DC::GetTopicVector<topicType>::GetTopicVector(const ros::NodeHandlePtr rosNode_pr,
    const std::string topic_name, const int vector_size_):
  vector_size(vector_size_){
    StateSub = rosNode_pr->subscribe(topic_name, 1, &GetTopicVector::StateCallback, this);
}

DC::DataCollection::DataCollection(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time_):
  rosNodeConstPtr(new ros::NodeHandle(node_name)),
  //force_sub_ptr(new DC::GetTopicVector<DC::FORCE_TYPE>(this->rosNodeConstPtr, DC::FORCE_NAME)),
  scan_sub_ptr(new DC::GetTopicVector<DC::SCAN_TYPE>(this->rosNodeConstPtr, DC::SCAN_NAME)),
  rgb_sub_ptr(new DC::GetTopicVector<DC::RGB_IMAGE_TYPE>(this->rosNodeConstPtr, DC::RGB_IMAGE_NAME)),
  depth_sub_ptr(new DC::GetTopicVector<DC::DEPTH_IMAGE_TYPE>(this->rosNodeConstPtr, DC::DEPTH_IMAGE_NAME)),
  sleeping_time(sleeping_time_){
    assert(rosNodeConstPtr->getParam("/TB3_WITH_ACTOR", actor_name));
    force_sub_ptr = std::make_shared<DC::GetTopicVector<DC::FORCE_TYPE>>(this->rosNodeConstPtr, "/"+actor_name+"/actor_vel");
    dc_Service = this->rosNodeConstPtr->advertiseService(service_name, &DataCollection::ServiceCallback, this);

}

bool DC::DataCollection::ServiceCallback(
    data_collection::DataCollectionService::Request& req,
    data_collection::DataCollectionService::Response& res){
  std::unique_lock<std::mutex> state_1_lock(topic_mutex);
  res.force = this->force_sub_ptr->StateVector.back();
  res.scan = this->scan_sub_ptr->StateVector.back();
  res.depth_image = *(this->depth_sub_ptr->StateVector.back());
  res.rgb_image = *(this->rgb_sub_ptr->StateVector.back());
}
