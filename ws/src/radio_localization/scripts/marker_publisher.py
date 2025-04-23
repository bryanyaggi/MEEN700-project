#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from radio_localization.wgs84_transformer import Wgs84Transformer
from radio_localization.radio_model import RadioModel

import utm

def getWgs84Transformer():
    if not rospy.has_param('origin'):
        rospy.logerr("Parameter 'origin' not found.")
        return

    originParam = rospy.get_param('origin')

    origin = PoseStamped()
    origin.pose.position.x = originParam['longitude']
    origin.pose.position.y = originParam['latitude']
    origin.pose.orientation.w = 1.0

    return Wgs84Transformer(origin)

class MarkerPublisher:
    def __init__(self):
        self.baseStationPublisher = rospy.Publisher('base_stations', MarkerArray, queue_size=1, latch=True)
        self.ringPublisher = rospy.Publisher('station_rings', MarkerArray, queue_size=1 )
        self.wgs84Tf = getWgs84Transformer()

        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base stations' not found.")
            return
        baseStations = rospy.get_param('base_stations')
        self.baseStationIds = []
        self.baseStationLocations = []
        self.baseStationAs = []
        self.baseStationNs = []
        for bs in baseStations:
            self.baseStationIds.append(bs['id'])
            latitude = bs['latitude']
            longitude = bs['longitude']
            points = self.wgs84Tf.wgs84_to_local_xy([(latitude, longitude)])
            self.baseStationLocations.append((points[0][0], points[0][1]))
            self.baseStationAs.append(bs['A'])
            self.baseStationNs.append(bs['N'])

        self.radioModel = RadioModel(self.baseStationLocations, self.baseStationAs, self.baseStationNs)    
        rospy.Subscriber('/radio_measurements', Float32MultiArray, self.radioMeasurementCallback)
        
        self.publishBaseStationMarkers() 

    def radioMeasurementCallback(self, msg):
        rssi = np.array([msg.data[i] for i in range(0, len(msg.data), 2)])
        print(rssi)
        rospy.loginfo(f"Updated radio measurements: {rssi}")

        self.publishRingMarkers(rssi)

    
    def publishBaseStationMarkers(self):
        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base_stations' not found.")
            return

        baseStations = rospy.get_param('base_stations')

        markerArray = MarkerArray()
        for bs in baseStations:
            marker = Marker()
            marker.id = bs['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            latitude = bs['latitude']
            longitude = bs['longitude']

            '''
            marker.header.frame_id = 'utm'
            easting, northing, zoneNumber, zoneLetter = utm.from_latlon(latitude, longitude)
            marker.pose.position.x = easting
            marker.pose.position.y = northing
            '''
            marker.header.frame_id = 'map'
            points = self.wgs84Tf.wgs84_to_local_xy([(latitude, longitude)])
            marker.pose.position.x = points[0][0]
            marker.pose.position.y = points[0][1]

            marker.pose.orientation.w = 1.0

            marker.scale.x = 10.0
            marker.scale.y = 10.0
            marker.scale.z = 10.0
            marker.color.r = 1.0
            marker.color.b = 0.0
            marker.color.g = 0.0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration(0)
            markerArray.markers.append(marker)

        self.baseStationPublisher.publish(markerArray)
    
    def publishRingMarkers(self, rssi):
        ringArray = MarkerArray()
        for i in range(len(self.baseStationIds)):
            ring = Marker()
            ring.id = self.baseStationIds[i]
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD

            ranges = self.radioModel.getRangesFromRssi(rssi)
            ring.header.frame_id = 'map'
            ring.pose.position.x = self.baseStationLocations[i][0]
            ring.pose.position.y = self.baseStationLocations[i][1]
            ring.pose.position.z = 0.1

            ring.pose.orientation.w = 1.0

            ring.scale.x = ranges[i] * 2.0
            ring.scale.y = ranges[i] * 2.0
            ring.scale.z = 1.0
            ring.color.r = 0.0
            ring.color.b = 1.0
            ring.color.g = 0.0
            ring.color.a = 0.5

            ring.lifetime = rospy.Duration(0)
            ringArray.markers.append(ring)

        self.ringPublisher.publish(ringArray)

if __name__ == '__main__':
    rospy.init_node('marker_publisher')
    MarkerPublisher()
    rospy.spin()
