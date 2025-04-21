#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TwistStamped
from dbw_fca_msgs.msg import SteeringReport
from std_msgs.msg import Float64MultiArray

from radio_localization.moving_averager import MovingAverager

class ControlProcessor:
    def __init__(self):
        # Calculate buffer size and get publish frequency
        if not rospy.has_param('rates'):
            rospy.logerr("Parameter 'rates' not found.")
            return
        rates = rospy.get_param('rates')
        period = 1 / rates['measurement']
        velocityBuffer = int(period * rates['velocity_raw'])
        steerBuffer = int(period * rates['steer_raw'])
        publishFrequency = rates['control_processed']

        # Get steering ratio
        if not rospy.has_param('vehicle'):
            rospy.logerr("Parameter 'vehicle' not found.")
            return
        vehicle = rospy.get_param('vehicle')
        self.steeringRatio = vehicle['steering_ratio']
        
        # Initialize message
        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 2

        # Set up moving averagers
        self.velocityMovingAverager = MovingAverager(buffer=velocityBuffer)
        self.steerMovingAverager = MovingAverager(buffer=steerBuffer)

        rospy.Subscriber('vehicle/twist', TwistStamped, self.velocityCallback)
        rospy.Subscriber('vehicle/steering_report', SteeringReport, self.steerCallback)
        self.pub = rospy.Publisher('control', Float64MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(1 / publishFrequency), self.timerCallback)

    def velocityCallback(self, msg):
        self.velocityMovingAverager.append(msg.twist.linear.x)

    def steerCallback(self, msg):
        steeringWheelAngle = msg.steering_wheel_angle
        roadWheelAngle = steeringWheelAngle / self.steeringRatio
        self.steerMovingAverager.append(roadWheelAngle)

    def timerCallback(self, event):
        self.msg.data[0] = self.velocityMovingAverager.getAverage()
        self.msg.data[1] = self.steerMovingAverager.getAverage()
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('yaw_measurement_processor')
    ControlProcessor()
    rospy.spin()
