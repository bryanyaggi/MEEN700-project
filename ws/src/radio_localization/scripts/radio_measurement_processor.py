#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class RadioMeasurementProcessor:
    def __init__(self):
        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base stations' not found.")
            return

        baseStations = rospy.get_param('base_stations')
        for bs in baseStations

        self.pub = rospy.Publisher('radio_distance', Float32MultiArray, queue_size=1)
        rospy.Subscriber('rssi', Int32MultiArray, self.rssiCallback)
        rospy.Subscriber('current_tof', Int32MultiArray, self.tofCallback)
        rospy.Timer(rospy.Duration(1), self.timerCallback)

    def rssiCallback(self, msg):
        pass

    def tofCallback(self, msg):
        pass

    def timerCallback(self, event):
        self.

if __name__ == '__main__':
    rospy.init_node('measurement_aggregator')
    ma = MeasurementAggregator()
    rospy.spin()
