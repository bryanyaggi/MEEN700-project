#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

class YawMeasurementProcessor:
    def __init__(self, publishFrequency=10):
        '''
        publishFrequency in seconds
        '''
        self.msg = Float64()

        rospy.Subscriber('localization/AbsolutePoseMeasurement', PoseWithCovarianceStamped, self.poseCallback)
        self.pub = rospy.Publisher('yaw_measurement', Float64, queue_size=1)
        rospy.Timer(rospy.Duration(1 / publishFrequency), self.timerCallback)

    def poseCallback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.msg.data = yaw

    def timerCallback(self, event):
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('yaw_measurement_processor')
    yp = YawMeasurementProcessor()
    rospy.spin()
