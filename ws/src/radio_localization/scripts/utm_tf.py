#!/usr/bin/env python3

import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

import tf
import utm

class UtmStaticTfPublisher:
    def __init__(self):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.msg = TransformStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "/utm"
        self.msg.child_frame_id = "/map"
        self.msg.transform.translation.z = 0
         
        # Use a quaternion to define the rotation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.transform.rotation.x = quaternion[0]
        self.msg.transform.rotation.y = quaternion[1]
        self.msg.transform.rotation.z = quaternion[2]
        self.msg.transform.rotation.w = quaternion[3]

        rospy.Subscriber("local_xy_origin", PoseStamped, self.originCallback)

    def originCallback(self, msg):
        easting, northing, zoneNumber, zoneLetter = utm.from_latlon(msg.pose.position.y, msg.pose.position.x)
        self.msg.transform.translation.x = easting
        self.msg.transform.translation.y = northing
        self.broadcaster.sendTransform(self.msg)

if __name__ == '__main__':
    rospy.init_node('utm_static_tf_publisher')
    UtmStaticTfPublisher()
    rospy.spin()