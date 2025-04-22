#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from radio_localization.moving_averager import MovingAverager

class YawMeasurementProcessor:
    def __init__(self, useMovingAverage=False):
        self.useMovingAverage = useMovingAverage

        # Calculate buffer size and get publish frequency
        if not rospy.has_param('rates'):
            rospy.logerr("Parameter 'rates' not found.")
            return
        rates = rospy.get_param('rates')
        period = 1 / rates['measurement']
        buffer = int(period * rates['yaw_raw'])
        publishFrequency = rates['yaw_processed']
        
        self.msg = Float64()

        self.movingAverager = MovingAverager(buffer=buffer)

        #rospy.Subscriber('localization/AbsolutePoseMeasurement', PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber('localization/map_pose_repub', Odometry, self.poseCallback)
        self.pub = rospy.Publisher('yaw_measurement', Float64, queue_size=1)
        rospy.Timer(rospy.Duration(1 / publishFrequency), self.timerCallback)

    def poseCallback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        if self.useMovingAverage:
            self.movingAverager.append(yaw)
        else:
            self.msg.data = yaw

    def timerCallback(self, event):
        if self.useMovingAverage:
            self.msg.data = self.movingAverager.getAverage()
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('yaw_measurement_processor')
    yp = YawMeasurementProcessor()
    rospy.spin()
