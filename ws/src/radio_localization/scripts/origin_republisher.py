#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class OriginRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/local_xy_origin', PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber('/local_xy_origin_original', PoseStamped, self.callback)

    def callback(self, msg):
        msg.header.frame_id = 'map'
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('origin_republisher')
    OriginRepublisher()
    rospy.spin()
