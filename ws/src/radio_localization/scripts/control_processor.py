#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        
        self.msg = Float64MultiArray()

        self.velocityMovingAverager = MovingAverager(buffer=velocityBuffer)
        self.steerMovingAverager = MovingAverager(buffer=steerBuffer)

        rospy.Subscriber('vehicle/twist', TwistStamped, self.velocityCallback)
        rospy.subscriber('vehicle/steering_report', SteeringReport, self.steerCallback)
        self.pub = rospy.Publisher('control', Float64MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(1 / publishFrequency), self.timerCallback)

    def velocityCallback(self, msg):
        self.movingAverager.append(msg.twist.)

    def timerCallback(self, event):
        self.msg.data = self.movingAverager.getAverage()
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('yaw_measurement_processor')
    yp = YawMeasurementProcessor()
    rospy.spin()
