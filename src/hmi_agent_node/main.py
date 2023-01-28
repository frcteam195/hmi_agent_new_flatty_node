#!/usr/bin/env python3

import rospy
from dataclasses import dataclass
from ck_ros_base_msgs_node.msg import Joystick_Status, Robot_Status
from ck_ros_msgs_node.msg import HMI_Signals
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.ckmath import *
from ck_ros_msgs_node.msg import Intake_Control, Led_Control
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from nav_msgs.msg._Odometry import Odometry
import numpy as np


@dataclass
class DriveParams:
    drive_fwd_back_axis_id: int = -1
    drive_fwd_back_axis_inverted: bool = False

    drive_left_right_axis_id: int = -1
    drive_left_right_axis_inverted: bool = False

    drive_z_axis_id: int = -1
    drive_z_axis_inverted: bool = False

    drive_axis_deadband: float = 0.05
    drive_z_axis_deadband: float = 0.05

    robot_orient_button_id: int = -1
    field_orient_button_id: int = -1
    brake_button_id: int = -1
    reset_odometry_button_id: int = -1

@dataclass
class OperatorParams:
    cube_request_button_id: int = -1
    cone_request_button_id: int = -1
    party_mode_button_id: int = -1


drive_params = DriveParams()
operator_params = OperatorParams()

hmi_pub = None
odom_pub = None
intake_pub = None
led_control_pub = None

led_timer = 0
party_time = True
led_control_msg = Led_Control()

drive_joystick = Joystick(0)
operator_controller = Joystick(1)

drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED

def joystick_callback(msg: Joystick_Status):
    global drivetrain_orientation
    global hmi_pub
    global led_control_pub

    global robot_status

    global drive_joystick
    global operator_params

    global drive_params
    global operator_params

    global led_timer
    global party_time
    global led_control_msg

    Joystick.update(msg)

    hmi_update_msg = HMI_Signals()

    hmi_update_msg.drivetrain_brake = drive_joystick.getButton(drive_params.brake_button_id)

    invert_axis_fwd_back = -1 if drive_params.drive_fwd_back_axis_inverted else 1
    invert_axis_left_right = -1 if drive_params.drive_left_right_axis_inverted else 1

    hmi_update_msg.drivetrain_fwd_back = invert_axis_fwd_back * drive_joystick.getFilteredAxis(drive_params.drive_fwd_back_axis_id, drive_params.drive_axis_deadband)

    hmi_update_msg.drivetrain_left_right = invert_axis_left_right * drive_joystick.getFilteredAxis(drive_params.drive_left_right_axis_id, drive_params.drive_axis_deadband)

    x = hmi_update_msg.drivetrain_fwd_back
    y = hmi_update_msg.drivetrain_left_right

    invert_axis_z = -1 if drive_params.drive_z_axis_inverted else 1
    z = invert_axis_z * drive_joystick.getFilteredAxis(drive_params.drive_z_axis_id, drive_params.drive_z_axis_deadband)

    r = hypotenuse(x, y)
    theta = polar_angle_rad(x, y)

    z = np.sign(z) * pow(z, 2)
    active_theta = theta
    if (r > drive_params.drive_axis_deadband):
        active_theta = theta

    hmi_update_msg.drivetrain_swerve_direction = active_theta
    hmi_update_msg.drivetrain_swerve_percent_fwd_vel = limit(r, 0.0, 1.0)
    hmi_update_msg.drivetrain_swerve_percent_angular_rot = z

    intake_control = Intake_Control()

    if drive_joystick.getButton(drive_params.robot_orient_button_id):
        # drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED
        intake_control.rollers_intake = True
        intake_control.rollers_outake = False
    elif drive_joystick.getButton(drive_params.field_orient_button_id):
        intake_control.rollers_intake = False 
        intake_control.rollers_outake = True 
    
    intake_pub.publish(intake_control)
    
        # drivetrain_orientation = HMI_Signals.FIELD_ORIENTED

    hmi_update_msg.drivetrain_orientation = drivetrain_orientation

    # LED Control
    led_control_msg.control_mode = Led_Control.ANIMATE
    led_control_msg.number_leds = 8
    
    if robot_status.get_mode() == RobotMode.DISABLED:
        led_control_msg.animation = Led_Control.STROBE
        led_control_msg.speed = 0.3
        led_control_msg.brightness = 0.5
        led_control_msg.red = 255
        led_control_msg.green = 0
        led_control_msg.blue = 0

    else:
        if operator_controller.getButton(operator_params.cone_request_button_id):
            led_timer = rospy.get_time()
            led_control_msg.animation = Led_Control.STROBE
            led_control_msg.speed = 0.1
            led_control_msg.brightness = 0.5
            led_control_msg.red = 255
            led_control_msg.green = 255
            led_control_msg.blue = 0

        if operator_controller.getButton(operator_params.cube_request_button_id):
            led_timer = rospy.get_time()
            led_control_msg.animation = Led_Control.STROBE
            led_control_msg.speed = 0.1
            led_control_msg.brightness = 0.5
            led_control_msg.red = 255
            led_control_msg.green = 0
            led_control_msg.blue = 255
        
        if operator_controller.getRisingEdgeButton(operator_params.party_mode_button_id):
            party_time = not party_time
            
        if rospy.get_time() - led_timer > 3:
            if not party_time:
                led_control_msg.animation = Led_Control.LARSON
                led_control_msg.speed = 0.5
                led_control_msg.brightness = 0.5
                led_control_msg.red = 0
                led_control_msg.green = 255
                led_control_msg.blue = 0
        
            else:
                led_control_msg.animation = Led_Control.RAINBOW
                led_control_msg.speed = 1
                led_control_msg.brightness = 1

    led_control_pub.publish(led_control_msg)

    if drive_joystick.getButton(drive_params.reset_odometry_button_id):
        odom = Odometry()

        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.orientation.w = 1
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = 0
        odom.pose.pose.position.x = 0
        odom.pose.pose.position.y = 0
        odom.pose.pose.position.z = 0

        odom.twist.twist.linear.x = 0
        odom.twist.twist.linear.y = 0
        odom.twist.twist.linear.z = 0

        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = 0

        odom.pose.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.00001,
        ]

        odom.twist.covariance =[
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
        ]

        odom_pub.publish(odom)

    hmi_pub.publish(hmi_update_msg)


def init_params():
    global drive_params
    global operator_params

    drive_params.drive_fwd_back_axis_id = rospy.get_param("/hmi_agent_node/drive_fwd_back_axis_id", -1)
    drive_params.drive_fwd_back_axis_inverted = rospy.get_param("/hmi_agent_node/drive_fwd_back_axis_inverted", False)

    drive_params.drive_left_right_axis_id = rospy.get_param("/hmi_agent_node/drive_left_right_axis_id", -1)
    drive_params.drive_left_right_axis_inverted = rospy.get_param("/hmi_agent_node/drive_left_right_axis_inverted", False)

    drive_params.drive_z_axis_id = rospy.get_param("/hmi_agent_node/drive_z_axis_id", -1)
    drive_params.drive_z_axis_inverted = rospy.get_param("/hmi_agent_node/drive_z_axis_inverted", False)

    drive_params.drive_z_axis_deadband = rospy.get_param("/hmi_agent_node/drive_z_axis_deadband", 0.05)
    drive_params.drive_axis_deadband = rospy.get_param("/hmi_agent_node/drive_axis_deadband", 0.05)

    drive_params.robot_orient_button_id = rospy.get_param("/hmi_agent_node/robot_orient_button_id", -1)
    drive_params.field_orient_button_id = rospy.get_param("/hmi_agent_node/field_orient_button_id", -1)
    drive_params.brake_button_id = rospy.get_param("/hmi_agent_node/brake_button_id", -1)
    drive_params.reset_odometry_button_id = rospy.get_param("hmi_agent_node/reset_odometry_button_id", -1)

    operator_params.cube_request_button_id = rospy.get_param("/hmi_agent_node/cube_request_button_id", -1)
    operator_params.cone_request_button_id = rospy.get_param("/hmi_agent_node/cone_request_button_id", -1)
    operator_params.party_mode_button_id = rospy.get_param("/hmi_agent_node/party_mode_button_id", -1)


def ros_main(node_name):
    global hmi_pub
    global odom_pub
    global intake_pub
    global led_control_pub
    rospy.init_node(node_name)
    init_params()

    register_for_robot_updates()

    hmi_pub = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
    odom_pub = rospy.Publisher(name="/ResetHeading", data_class=Odometry, queue_size=10, tcp_nodelay=True)
    intake_pub = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
    led_control_pub = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)

    rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=joystick_callback, queue_size=1, tcp_nodelay=True)
    rospy.spin()
