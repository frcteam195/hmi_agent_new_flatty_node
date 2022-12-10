#!/usr/bin/env python3

import rospy
from dataclasses import dataclass
from rio_control_node.msg import Joystick_Status, Robot_Status
from hmi_agent_node.msg import HMI_Signals
from ck_utilities_py_node.joystick import Joystick

@dataclass
class Params:
    drivetrain_fwd_back : float = 0
    drivetrain_left_right : float = 0
    drivetrain_swerve_percent_fwd_vel : float = 0
    drivetrain_swerve_direction : float = 0
    drivetrain_swerve_percent_angular_rot : float = 0
    drivetrain_quickturn : bool = False
    drivetrain_brake : bool = False
    gauge_axis_id: int = -1
    elevator_vertical_axis_id: int = -1
    claw_open_button_id: int = -1

params = Params()

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

    hmi_update_msg.gauge_value = int(1500 + (1500 * drive_joystick.getRawAxis(params.gauge_axis_id)))
    hmi_update_msg.elevator_vertical = float(drive_joystick.getRawAxis(params.elevator_vertical_axis_id))
    hmi_update_msg.claw_open = bool(drive_joystick.getButton(params.claw_open_button_id))

    hmi_pub.publish(hmi_update_msg)


def init_params():
    global params
    params.gauge_axis_id = rospy.get_param("gauge_axis_id", -1)
    params.elevator_vertical_axis_id = rospy.get_param("elevator_vertical_axis_id", -1)
    params.claw_open_button_id = rospy.get_param("claw_open_button_id", -1)


def ros_main(node_name):
    rospy.init_node(node_name)
    init_params()
    rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=joystick_callback, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(name="/RobotStatus", data_class=Robot_Status, callback=robot_status_callback, queue_size=1, tcp_nodelay=True)
    rospy.spin()