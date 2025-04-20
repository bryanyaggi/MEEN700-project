#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from dbw_fca_msgs.msg import SteeringReport

class LocalizationFilterNode:
    def __init__(self):
        self.yaw = 0
        self.steeringAngle = 0
        self.velocity = 0

        # Publish estimate with covariance
        self.pub = rospy.Publisher('radio_localization_pose', PoseWithCovarianceStamped, queue=1)
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = 'utm'

        # Subscribe to measurements
        rospy.Subscriber('radio_measurements', Float32MultiArray, self.radioMeasurementsCallback)
        rospy.Subscriber('localization/AbsolutePoseMeasurement', PoseWithCovarianceStamped, self.yawMeasurementCallback)
        
        # Subscribe to control
        rospy.Subscriber('vehicle/twist', TwistStamped, self.controlCallback)
        rospy.subscriber('vehicle/steering_report', SteeringReport, self.controlCallback)

    def radioMeasurementsCallback(self, msg):
        self.pub.publish()

    def yawMeasurementCallback(self, msg):
        pass

if __name__ == '__main__':
    rospy.init_node('localization_filter')
    lfn = LocalizationFilterNode()
