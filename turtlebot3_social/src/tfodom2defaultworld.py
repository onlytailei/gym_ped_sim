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
import numpy as np

if __name__ == '__main__':
    
    rospy.init_node('tfodom2defaultworld')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(),
                "odom",
                "default_world")
        rate.sleep()
