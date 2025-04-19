#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class OdometryRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/localization/utm_pose_repub', Odometry, queue_size=10)
        rospy.Subscriber('/localization/utm_pose', Odometry, self.callback)

    def callback(self, msg):
        msg.header.frame_id = 'utm'
        msg.pose.pose.position.z = 0
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('odometry_republisher')
    OdometryRepublisher()
    rospy.spin()
