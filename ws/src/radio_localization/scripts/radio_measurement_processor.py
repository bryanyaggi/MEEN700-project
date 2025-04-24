#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from radio_localization.silvus_constants import DATA_LENGTH_PER_RADIO, INVALID_RSSI, INVALID_TOF

def parseRssiMsg(msg):
    nodeId = msg.data[0]
    rssis = msg.data[1:]
    avgRssi = sum(rssis) / len(rssis)
    return nodeId, avgRssi

class RadioMeasurementProcessor:
    def __init__(self):
        '''
        ageThreshold in milliseconds
        '''
        # Get base station IDs
        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base stations' not found.")
            return
        baseStations = rospy.get_param('base_stations')
        self.baseStationIds = []
        for bs in baseStations:
            self.baseStationIds.append(bs['id'])

        # Get publish frequency
        if not rospy.has_param('rates'):
            rospy.logerr("Parameter 'rates' not found.")
            return
        rates = rospy.get_param('rates')
        publishFrequency = rates['measurement']

        self.ageThreshold = 1000 / publishFrequency # period in milliseconds

        # Initialize message
        self.msg = Float32MultiArray()
        self.msg.data = [INVALID_RSSI, INVALID_TOF] * len(self.baseStationIds)

        # Set up pub/sub
        self.pub = rospy.Publisher('radio_measurements', Float32MultiArray, queue_size=1)
        rospy.Subscriber('rssi', Int32MultiArray, self.rssiCallback)
        rospy.Subscriber('current_tof', Int32MultiArray, self.tofCallback)
        rospy.Timer(rospy.Duration(1 / publishFrequency), self.timerCallback)

    def rssiCallback(self, msg):
        nodeId, rssi = parseRssiMsg(msg)

        # Discard unexpected radios
        if nodeId not in self.baseStationIds:
            return

        # Store value in message
        idIndex = self.baseStationIds.index(nodeId)
        self.msg.data[idIndex * DATA_LENGTH_PER_RADIO] = rssi

    def tofCallback(self, msg):
        for i in range(0, len(msg.data), 3):
            nodeId = msg.data[i]
            
            # Discard if unexpected radios
            if nodeId not in self.baseStationIds:
                return

            # Store value in message
            idIndex = self.baseStationIds.index(nodeId)
            tof = msg.data[i + 1]
            if msg.data[i + 2] > self.ageThreshold:
                tof = -1
            self.msg.data[idIndex * DATA_LENGTH_PER_RADIO + 1] = tof

    def timerCallback(self, event):
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('radio_measurement_processor')
    ma = RadioMeasurementProcessor()
    rospy.spin()
