#!/usr/bin/env python3
from email import header
import os
import rospy
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from duckietown.dtros import DTROS, NodeType

import tf

######Subscribers
# /duckiedrone/lidar_sensor_node  1 thru 4
# /duckiedrone/infrared_node
# /duckiedrone/imu

######Publishers
# /duckiedrone/altitude     #todo rename /pidrone/rangefinder


#this node takes in the four lidar sensor data
#streams and outputs the average of the most recent
#four measurments 

#the purpose of this is to stop one sensor over a watchtower
#or other discontinous terrain to throw off the height estimation
class RangeFinderAverageNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(RangeFinderAverageNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.ranges = np.array([0.0])
        self.range_index = 0
        self.max_range = 0.0
        self.min_range = 0.0
        self.header = None #todo cread frame ID for this header

        # by default: angle between the direction the range finder is pointed (normal to the drone)
        #             and the gravity vector (towards the ground)
        self.range_angle = 0

        self._sub_imu = rospy.Subscriber('imu', Imu, self.update_angle, queue_size=1) # do not increase the queue size
        self._sub_tof = rospy.Subscriber('altitude_tof_driver_node/range', Range, self.callback, queue_size=1) # do not increase the queue size

        self._pub = rospy.Publisher('altitude_node', Range, queue_size=1) # is not tacking into account the center of mass of the drone
        self._heartbeat = rospy.Publisher('heartbeat/altitude_node', Empty, queue_size=1)

        self._timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.cb_timer)

    def update_angle(self, imu_msg):

        #for more on quaternion see the ros tf page, or there is a great
        #three blue one brown mini-series on them.

        # however the imu gives a quaternion q and by transfomring the gravity vector
        # g=[0,0,1,0] in quaternion space, using g' = q dot g dot q_conjugate you get
        # g' which is the vector going upwards out of the plane of the body of the drone
        # and using arctan(g dot g') we are able to find the angle that the drone makes
        # with gravity, and therefore what factor to multiply the rangefinder data to convert
        # it to distance off the ground.

        base = np.array([0,0,1,0])
        quat = imu_msg.orientation
        quaternion = np.array([quat.x, quat.y, quat.z, quat.w])
        quaternion_conjugate = np.array([-quat.x, -quat.y, -quat.z, quat.w])

        new = tf.transformations.quaternion_multiply(
            quaternion,
            tf.transformations.quaternion_multiply(
                base,
                quaternion_conjugate,
            ),
        )

        self.range_angle = np.arccos(np.dot(new[0:3], base[0:3]))

    def increment_index(self):
        self.range_index += 1
        self.range_index = self.range_index % len(self.ranges)

    def return_median(self):
        return np.median(self.ranges)

    def return_mean(self):
        return np.mean(self.ranges)

    def callback(self, range_msg):
        #read in data
        self.ranges[self.range_index] = range_msg.range * np.cos(self.range_angle)
        #project this along the verticle axis TODO: convert this to use frames
        self.increment_index()
        if self.max_range == None or self.max_range > range_msg.max_range:
            self.max_range = range_msg.max_range
        self.min_range = range_msg.min_range
        self.header = range_msg.header

    def cb_timer(self, event=None):
        msg = Range()

        msg.max_range = self.max_range
        msg.min_range = self.min_range
        msg.range = self.return_median()
        if self.header is not None:
            msg.header = self.header

        self._pub.publish(msg)
        self._heartbeat.publish(Empty())
        #print "Last four ranges recived: ", np.around(self.ranges, 4), "\t\t\t\r"


if __name__ == "__main__":
    range_finder_node = RangeFinderAverageNode("altitude_node")
    rospy.spin()
