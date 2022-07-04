#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import  Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import DroneControl as RC
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

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
        self.pid.output_limits = (-100, 400)

        self._altitude_sub = rospy.Subscriber(
            "altitude_node/altitude", Range, self.altitude_data_callback, queue_size=1)
        self.cmd_publisher = rospy.Publisher("fly_commands_mux_node/autonomous", RC, queue_size=1)
        self.pid_publisher = rospy.Publisher("alt_controller_node/pid", Vector3, queue_size=1)
        self.err_publisher = rospy.Publisher("alt_controller_node/err", Float32, queue_size=1)

    def altitude_data_callback(self, msg):
        control = self.pid(msg.range)

        cmd_msg = RC()
        cmd_msg.roll = 1500
        cmd_msg.pitch = 1500
        cmd_msg.yaw = 1500
        cmd_msg.throttle = 1100+ control
        p,i,d=self.pid.components
        err = self.ALTITUDE_SETPOINT - msg.range
        print(f"Error: {(self.ALTITUDE_SETPOINT-msg.range)}, P: {p}, I: {i}, D: {d}")

        pid_msg = Vector3()
        pid_msg.x = p
        pid_msg.y = i
        pid_msg.z = d
        self.pid_publisher.publish(pid_msg)

        err_msg = Float32()
        err_msg.data = err
        self.err_publisher.publish(err_msg)

        self.cmd_publisher.publish(cmd_msg)

    def on_shutdown(self):
        self.stop()


if __name__ == "__main__":
    # Initialize the node
    alt_controller_node = AltitudeControllerNode(
        node_name="alt_controller_node")
    # Keep it spinning
    rospy.spin()
