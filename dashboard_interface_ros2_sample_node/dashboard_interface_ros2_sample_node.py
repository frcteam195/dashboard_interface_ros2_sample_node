#!/usr/bin/python3
import signal
import sys

import rclpy
from dashboard_interface_ros2_sample_node.generated.parameters import ParameterizedNode

from std_msgs.msg import String

from threading import Thread

import json
import socket
from frc_robot_utilities_ros2_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_ros2_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy

from ck_ros2_msgs_node.msg import HealthMonitorStatus, HealthMonitorControl, AutonomousConfiguration, AutonomousSelection

BUFFER_SIZE = 1024

class LocalNode(ParameterizedNode):
    def __init__(self):
        super().__init__('dashboard_interface_ros2_sample_node')
        register_for_robot_updates()
        self.autonomous_configuration_subscriber = BufferedROSMsgHandlerPy(AutonomousConfiguration)
        self.autonomous_configuration_subscriber.register_for_updates("AutonomousConfiguration")

        self.health_status_subscriber = BufferedROSMsgHandlerPy(HealthMonitorStatus)
        self.health_status_subscriber.register_for_updates("HealthMonitorStatus")

        self.acknowledge_publisher = self.create_publisher(topic="HealthMonitorControl", msg_type=HealthMonitorControl, qos=10)
        self.autonomous_selection_publisher = self.create_publisher(topic="AutonomousSelection", msg_type=AutonomousSelection, qos=10)

        self.clients = []

        self.get_logger().info(f"UDP target IP: {self.Parameters.udp_listen_address}")
        self.get_logger().info(f"UDP target port: {self.Parameters.udp_receive_port}")

        self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.dashboard_socket.setblocking(0)
        self.dashboard_socket.bind((self.Parameters.udp_listen_address, self.Parameters.udp_receive_port))

        self.loop_thread = Thread(target=self.loop)
        self.loop_rate = self.create_rate(10, self.get_clock())

    def loop(self) -> None:
        """
        Periodic function for the dashboard interface node.
        """
        while rclpy.ok():

            try:
                buffer, (address, _) = self.dashboard_socket.recvfrom(BUFFER_SIZE)
                message = json.loads(buffer)

                if (address, self.Parameters.udp_send_port) not in self.clients:
                    self.get_logger().info(f"New Client: {address}")
                    self.clients.append((address, self.Parameters.udp_send_port))

                if message["type"] == "data":
                    selection_message = AutonomousSelection()
                    selection_message.autonomous = message["autonomous"]["autonomous"]
                    self.autonomous_selection_publisher.publish(selection_message)

                    if message["acknowledge"]:
                        acknowledge_message = HealthMonitorControl()
                        acknowledge_message.acknowledge = True
                        self.acknowledge_publisher.publish(acknowledge_message)

            except:
                pass

            self.send_dashboard_packet()

            self.loop_rate.sleep()

    def send_dashboard_packet(self):
        """
        Sends a packet of data to the dashboard.
        """
        robot_status_data = ""
        if robot_status is not None:
            robot_status_data = robot_status.get_message()

        hmi_updates_data = ""
        if hmi_updates.get() is not None:
            hmi_updates_data = {
                "drivetrain_forward_back": hmi_updates.get().drivetrain_fwd_back,
                "drivetrain_left_right": hmi_updates.get().drivetrain_left_right,
                "drivetrain_swerve_direction": hmi_updates.get().drivetrain_swerve_direction,
                "drivetrain_heading": hmi_updates.get().drivetrain_heading,
                "drivetrain_brake": hmi_updates.get().drivetrain_brake,
                "drivetrain_orientation": hmi_updates.get().drivetrain_orientation
            }

        autonomous_configuration = None
        if self.autonomous_configuration_subscriber.get() is not None:
            autonomous_configuration_message : AutonomousConfiguration = self.autonomous_configuration_subscriber.get()
            autonomous_configuration = autonomous_configuration_message.autonomous_options

        faults = []
        ros_is_booted = False
        if self.health_status_subscriber.get() is not None:
            health_status_message : HealthMonitorStatus = self.health_status_subscriber.get()
            for fault in health_status_message.faults:
                faults.append({"code": fault.code, "priority": fault.priority})
            ros_is_booted = health_status_message.is_ros_fully_booted

        packet = {
            "robot_status": robot_status_data,
            "hmi_updates": hmi_updates_data,
            "autonomous_configuration": autonomous_configuration,
            "faults": faults,
            "ros_booted" : ros_is_booted
        }

        for client in self.clients:
            self.dashboard_socket.sendto(json.dumps(packet).encode("utf-8"), client)


        
def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = LocalNode()
    node.loop_thread.start()
    rclpy.spin(node)
    rclpy.shutdown()
    node.loop_thread.join(5)


if __name__ == '__main__':
    main()