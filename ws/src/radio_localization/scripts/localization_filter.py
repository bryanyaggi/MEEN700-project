#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import tf

from marker_publisher import getWgs84Transformer
from radio_localization.localization_filter import LocalizationFilter

class LocalizationFilterNode:
    def __init__(self):
        self.yaw = 0
        self.velocity = 0
        self.steeringAngle = 0

        # Initialize pose message
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = 'map'
        state = np.array([0, 0, 0])
        self.updateMsgWithState(state)
        covariance = np.diag([1e6, 1e6, 1.])
        self.updateMsgWithCovariance(covariance)

        # Get vehicle parameters
        if not rospy.has_param('vehicle'):
            rospy.logerr("Parameter 'vehicle' not found.")
            return
        vehicle = rospy.get_param('vehicle')
        wheelbase = vehicle['wheelbase']

        # Get WGS84 transformer
        wgs84Tf = getWgs84Transformer()

        # Get base station parameters
        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base stations' not found.")
            return
        baseStations = rospy.get_param('base_stations')
        self.baseStationIds = []
        baseStationLocations = []
        baseStationAs = []
        for bs in baseStations:
            self.baseStationIds.append(bs['id'])
            latitude = bs['latitude']
            longitude = bs['longitude']
            points = wgs84Tf.wgs84_to_local_xy([(latitude, longitude)])
            baseStationLocations.append((points[0][0], points[0][1]))
            baseStationAs.append(bs['A'])

        # Create filter
        self.filter = LocalizationFilter(wheelbase, baseStationLocations, baseStationAs)

        # Publish estimate with covariance
        self.pub = rospy.Publisher('radio_localization_pose', PoseWithCovarianceStamped, queue_size=1)

        # Subscribe to measurements
        rospy.Subscriber('radio_measurements', Float32MultiArray, self.radioMeasurementsCallback)
        rospy.Subscriber('yaw_measurement', Float64, self.yawMeasurementCallback)
        
        # Subscribe to control
        rospy.Subscriber('control', Float64MultiArray, self.controlCallback)

    def updateMsgWithState(self, state):
        self.msg.pose.pose.position.x = state[0]
        self.msg.pose.pose.position.y = state[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, state[2])
        self.msg.pose.pose.orientation.x = quaternion[0]
        self.msg.pose.pose.orientation.y = quaternion[1]
        self.msg.pose.pose.orientation.z = quaternion[2]
        self.msg.pose.pose.orientation.w = quaternion[3]

    def updateMsgWithCovariance(self, covariance):
        # x row
        self.msg.pose.covariance[0:2] = covariance[0, 0:2]
        self.msg.pose.covariance[5] = covariance[0, 2]
        # y row
        self.msg.pose.covariance[6:8] = covariance[1, 0:2]
        self.msg.pose.covariance[11] = covariance[1, 2]
        # yaw row
        self.msg.pose.covariance[30:32] = covariance[2, 0:2]
        self.msg.pose.covariance[35] = covariance[2, 2]

    def radioMeasurementsCallback(self, msg):
        self.pub.publish(self.msg)

    def yawMeasurementCallback(self, msg):
        self.yaw = msg.data

    def controlCallback(self, msg):
        self.velocity = msg.data[0]
        self.steeringAngle = msg.data[1]

if __name__ == '__main__':
    rospy.init_node('localization_filter')
    LocalizationFilterNode()
    rospy.spin()
