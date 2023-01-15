#!/usr/bin/env python3

import rospy
from dataclasses import dataclass
from rio_control_node.msg import Joystick_Status, Robot_Status
from ck_ros_msgs_node.msg import HMI_Signals
from ck_utilities_py_node.joystick import Joystick

@dataclass
class DriveParams:
    drive_fwd_back_axis_id: int = -1
    drive_fwd_back_axis_inverted: bool = False
    drive_turn_axis_id: int = -1
    drive_turn_axis_inverted: bool = False
    drive_z_axis_id: int = -1
    drive_z_axis_inverted: bool = False
    drive_axis_deadband: float = 0.05
    drive_z_axis_deadband: float = 0.05

    drive_brake_button_id: int = -1
    drive_quickturn_button_id: int = -1

drive_params = DriveParams()

hmi_pub = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)

drive_joystick = Joystick(0)
arm_joystick = Joystick(1)
bb1_joystick = Joystick(2)
bb2_joystick = Joystick(3)

is_auto = False

def robot_status_callback(msg : Robot_Status):
    global is_auto
    is_auto = (msg.robot_state == msg.AUTONOMOUS)

def joystick_callback(msg : Joystick_Status):
    global is_auto
    global hmi_pub
    global drive_joystick
    global arm_joystick
    global bb1_joystick
    global bb2_joystick
    global params
    Joystick.update(msg)

    hmi_update_msg = HMI_Signals()

    if drive_params.drive_fwd_back_axis_inverted:
        hmi_update_msg.drivetrain_fwd_back = -drive_joystick.getRawAxis(drive_params.drive_fwd_back_axis_id)
    else:
        hmi_update_msg.drivetrain_fwd_back = drive_joystick.getRawAxis(drive_params.drive_fwd_back_axis_id)

    if drive_params.drive_turn_axis_inverted:
        hmi_update_msg.drivetrain_left_right = -drive_joystick.getRawAxis(drive_params.drive_turn_axis_id)
    else:
        hmi_update_msg.drivetrain_left_right = drive_joystick.getRawAxis(drive_params.drive_turn_axis_id)

    hmi_pub.publish(hmi_update_msg)


def init_params():
    global drive_params
    drive_params.drive_fwd_back_axis_id = rospy.get_param("drive_fwd_back_axis_id", -1)
    drive_params.drive_fwd_back_axis_inverted = rospy.get_param("drive_fwd_back_axis_inverted", -1)

    drive_params.drive_turn_axis_id = rospy.get_param("drive_turn_axis_id", -1)
    drive_params.drive_turn_axis_inverted = rospy.get_param("drive_turn_axis_inverted", -1)

def ros_main(node_name):
    rospy.init_node(node_name)
    init_params()
    rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=joystick_callback, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(name="/RobotStatus", data_class=Robot_Status, callback=robot_status_callback, queue_size=1, tcp_nodelay=True)
    rospy.spin()