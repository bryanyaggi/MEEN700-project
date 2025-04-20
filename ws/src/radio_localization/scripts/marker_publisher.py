#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from radio_localization.wgs84_transformer import Wgs84Transformer

import utm

class MarkerPublisher:
    def __init__(self):
        self.baseStationPublisher = rospy.Publisher('base_stations', MarkerArray, queue_size=1, latch=True)

    def publishBaseStationMarkers(self):
        if not rospy.has_param('base_stations'):
            rospy.logerr("Parameter 'base_stations' not found.")
            return

        baseStations = rospy.get_param('base_stations')

        markerArray = MarkerArray()
        for bs in baseStations:
            marker = Marker()
            marker.header.frame_id = 'utm'
            marker.id = bs['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            latitude = bs['latitude']
            longitude = bs['longitude']
            easting, northing, zoneNumber, zoneLetter = utm.from_latlon(latitude, longitude)
            marker.pose.position.x = easting
            marker.pose.position.y = northing
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

if __name__ == '__main__':
    rospy.init_node('marker_publisher')
    mp = MarkerPublisher()
    mp.publishBaseStationMarkers()
    rospy.spin()
