#!/usr/bin/env python
# coding=utf-8
'''
Author:Tai Lei
Date:Thu 09 Feb 2017 04:08:17 PM CST
Info:
    '''

#!/usr/bin/env python  
import rospy
import tf
import numpy as np
from gazebo_msgs.msg import ModelStates

class MountTB2Ped(object):
    
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.actor_name = rospy.get_param("TB3_WITH_ACTOR")
        self.actor_number = rospy.get_param("ACTOR_NUMBER")
        self.robot_pose = None
        self.robot_quat = None
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        

    def callback(self, data):
        for item in xrange(self.actor_number):
            actor_idx_ = data.name.index(self.actor_name[:-1]+str(item))
            actor_pose_ = data.pose[actor_idx_].position
            actor_quat_ = self.quat_trans(data.pose[actor_idx_].orientation)
            self.br.sendTransform((actor_pose_.x, actor_pose_.y, 0),
                    (actor_quat_.x, actor_quat_.y, actor_quat_.z, actor_quat_.w),
                    rospy.Time.now(),
                    self.actor_name[:-1]+str(item),
                    "default_world")
            print self.actor_name[:-1]+str(item)
            #if item == str(self.actor_name[-1]):
                #self.robot_pose = actor_pose_
                #self.robot_quat = actor_quat_
        
        #actor_idx = data.name.index(self.actor_name)
        #actor_pose = data.pose[actor_idx].position
        #actor_orien = data.pose[actor_idx].orientation
        #actor_pose.z = 0.0
        #quat_ = self.quat_trans(actor_orien) 
        #x = actor_orien.y
        #z = actor_orien.x
        #y = actor_orien.z
        #actor_orien.y = actor_orien.x
        #actor_orien.x = x
        #actor_orien.y = y
        #actor_orien.z = z
        
        #self.tb3modelstate.pose.position = self.robot_pose
        #self.tb3modelstate.pose.orientation = self.robot_quat
        ##self.model_set(self.tb3modelstate)
        #self.model_set.publish(self.tb3modelstate) 
        #self.br.sendTransform((0,0,0),
                #(0, 0, 0, 1),
                #rospy.Time.now(),
                #"odom",
                #self.actor_name)
    
    def quat_trans(self, quat):
        euler = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        quat_ = tf.transformations.quaternion_from_euler(euler[0]-0.5*np.pi, euler[1], euler[2]-0.5*np.pi) 
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]
        return quat

if __name__ == '__main__':
    
    rospy.init_node('tf2defaultworld')
    MountTB2Ped()
    rospy.spin()
    
    #target_x = -0.5
    #target_y = -5
    
    #br = tf.TransformBroadcaster()
    #rate = rospy.Rate(100)
    #while not rospy.is_shutdown():
        #target_x = rospy.get_param("TARGET_X")
        #target_y = rospy.get_param("TARGET_Y")
        #br.sendTransform((target_x, target_y, 0.0),
                #(0.0, 0.0, 0.0, 1.0),
                #rospy.Time.now(),
                #"target_pose",
                #"default_world")
        #rate.sleep()
