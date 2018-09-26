#!/usr/bin/env python
# coding=utf-8
'''
Author:Tai Lei
Date:Thu 09 Feb 2017 04:08:17 PM CST
Info:
    '''

#!/usr/bin/env python  
import roslib
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np

class MountTB2Ped(object):

    def __init__(self):
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.br = tf.TransformBroadcaster()
        self.model_set = rospy.Publisher("/gazebo/set_model_state", ModelState,queue_size=1)

        self.tb3modelstate = ModelState()
        self.tb3modelstate.model_name="turtlebot3_burger"
        self.actor_name = rospy.get_param("TB3_WITH_ACTOR")
    def callback(self, data):
        #tb3_idx = data.name.index("turtlebot3_burger")
        actor_idx = data.name.index(self.actor_name)
        #tb3_pose = data.pose[tb3_idx].position
        #tb3_orien = data.pose[tb3_idx].orientation
        #br.sendTransform((tb3_pose.x, tb3_pose.y, tb3_pose.z),
                #(tb3_orien.x, tb3_orien.y, tb3_orien.z, tb3_orien.w),
                #rospy.Time.now(),
                #"tb3",
                #"default_world")
        actor_pose = data.pose[actor_idx].position
        actor_orien = data.pose[actor_idx].orientation
        actor_pose.z = 0.0
        quat_ = self.quat_trans(actor_orien)
        #x = actor_orien.y
        #z = actor_orien.x
        #y = actor_orien.z
        #actor_orien.y = actor_orien.x
        #actor_orien.x = x
        #actor_orien.y = y
        #actor_orien.z = z

        self.tb3modelstate.pose.position =  actor_pose
        self.tb3modelstate.pose.orientation = quat_
        #self.model_set(self.tb3modelstate)
        self.model_set.publish(self.tb3modelstate)
        self.br.sendTransform((0,0,0),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "odom",
                self.actor_name)

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
