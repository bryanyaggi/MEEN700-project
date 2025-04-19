#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class MeasurementAggregator:
    def __init__(self):
        self.pub = rospy.Publisher('measurement', Float32MultiArray, queue_size=1)
        rospy.Subscriber('rssi', Int32MultiArray, self.rssiCallback)
        rospy.Subscriber('current_tof', Int32MultiArray, self.tofCallback)
        #rospy.Subscriber('current_tof', Int32MultiArray, self.tofCallback) # yaw
        rospy.Timer(rospy.Duration(1), self.timerCallback)

    def rssiCallback(self, msg):
        pass

    def tofCallback(self, msg):
        pass

    def yawCallback(self, msg):
        pass

    def timerCallback(self, event):
        self.

if __name__ == '__main__':
    rospy.init_node('measurement_aggregator')
    ma = MeasurementAggregator()
    rospy.spin()
