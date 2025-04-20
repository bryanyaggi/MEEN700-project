#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

import tf
import utm

class OriginNavSatFixPublisher:
    def __init__(self):
        self.msg = NavSatFix()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "map"

        self.pub = rospy.Publisher('origin_navsatfix', NavSatFix, queue_size=1, latch=True)
        rospy.Subscriber("local_xy_origin", PoseStamped, self.originCallback)

    def originCallback(self, msg):
        self.msg.latitude = msg.pose.position.y
        self.msg.longitude = msg.pose.position.x
        self.msg.altitude = msg.pose.position.z
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('origin_navsatfix_publisher')
    OriginNavSatFixPublisher()
    rospy.spin()
