#!/usr/bin/env python
# coding=utf-8
'''
Author:Tai Lei
Date:Di 15 Aug 2017 20:44:24 CEST
Info:
    '''
import rospy
import h5py
import time
import numpy as np
from data_collection.srv import DataCollectionService
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
import cv2
import os

class SAVE_DATA2H5PY(object):

    def __init__(self):
        rospy.init_node('saving_data_h5py', log_level=rospy.INFO)
        rospy.wait_for_service("/data_collection/data_collection_service")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.service_client = rospy.ServiceProxy(
                "/data_collection/data_collection_service",
                DataCollectionService)
        self.save_path = rospy.get_param("/SAVE_FILE")
        self.save_rate = rospy.get_param("/SAVE_RATE")
        self.save_count = rospy.get_param("/SAVE_COUNT")
        self.rate = rospy.Rate(self.save_rate)
        self.bridge = CvBridge()
        time.sleep(15)
    def save_data(self):
        count_ = 0
        while (not rospy.is_shutdown()) and (count_<self.save_count):
            ept_ = Empty()
            all_info = self.service_client(ept_)
            self.process(all_info,count_)
            self.rate.sleep()
            count_ += 1 
        self.service_client.close() 

    def process(self, all_msgs_, count_):
        name_file = self.save_path+os.sep+str(count_)
        h5_file_ = h5py.File(name_file,"w")
        velforce = all_msgs_.force

        h5_file_['x_force'] = velforce.angular.x
        h5_file_['y_force'] = velforce.angular.y 
        h5_file_['x_target'] = velforce.linear.x 
        h5_file_['y_target'] = velforce.linear.y 
        h5_file_['x_sf'] = velforce.linear.z 
        h5_file_['y_sf'] = velforce.angular.z 
        #h5_file_['yaw'] = velforce.angular.y 
        rospy.logerr("=======force %lf, %lf", velforce.angular.x, velforce.angular.y)

        scan = all_msgs_.scan.ranges
        h5_file_['ranges'] = np.array(scan) 

        rgb_image = self.bridge.imgmsg_to_cv2(all_msgs_.rgb_image, "rgb8")
        h5_file_['rbg_image'] = rgb_image
        
        #import matplotlib.pyplot as plt
        #plt.imshow(rgb_image)
        #plt.show()

        depth_image = self.bridge.imgmsg_to_cv2(all_msgs_.depth_image, "passthrough")
        h5_file_['depth_image'] = depth_image
        
        #plt.imshow(depth_image)
        #plt.show()

        h5_file_.close()

if __name__ == "__main__":
    save_data_obj = SAVE_DATA2H5PY()
    save_data_obj.save_data()
