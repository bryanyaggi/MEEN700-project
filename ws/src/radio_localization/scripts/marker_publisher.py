#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from radio_localization.wgs84_transformer import Wgs84Transformer

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
        self.wgs84Tf = getWgs84Transformer()
        self.publishBaseStationMarkers()

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

if __name__ == '__main__':
    rospy.init_node('marker_publisher')
    MarkerPublisher()
    rospy.spin()
