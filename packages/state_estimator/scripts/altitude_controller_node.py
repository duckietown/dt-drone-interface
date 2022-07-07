#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import  Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import DroneControl as RC

from simple_pid import PID


class AltitudeControllerNode(DTROS):
    
    ALTITUDE_SETPOINT = 0.3

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(AltitudeControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
            help=""
        )

        veh_name = rospy.get_namespace().rstrip('/')

        self.pid = PID(160, 110, 30, setpoint=self.ALTITUDE_SETPOINT)
        self.pid.output_limits = (0, 400)

        self._altitude_sub = rospy.Subscriber(
            "altitude_node/altitude", Range, self.altitude_data_callback, queue_size=1)
        self.cmd_publisher = rospy.Publisher("fly_commands_mux_node/autonomous", RC, queue_size=1)

    def altitude_data_callback(self, msg):
        control = self.pid(msg.range)

        cmd_msg = RC()
        cmd_msg.roll = 1500
        cmd_msg.pitch = 1500
        cmd_msg.yaw = 1500
        cmd_msg.throttle = 1100 + control
        p,i,d=self.pid.components
        print(f"Error: {(self.ALTITUDE_SETPOINT-msg.range)}, P: {p}, I: {i}, D: {d}")

        self.cmd_publisher.publish(cmd_msg)


if __name__ == "__main__":
    # Initialize the node
    alt_controller_node = AltitudeControllerNode(
        node_name="alt_controller_node")
    # Keep it spinning
    rospy.spin()
