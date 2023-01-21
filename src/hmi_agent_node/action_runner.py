#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread
from typing import List

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from hmi_agent_node.default_actions.SeriesAction import SeriesAction

active_action:SeriesAction = None

def start_action(action:SeriesAction):
    global active_action
    active_action = action
    if active_action is not None:
        try:
            active_action.start()
        except:
            rospy.logerr("Exception encountered starting action")
            active_action = None

def ros_func():
    global hmi_updates
    global robot_status

    global active_action

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if robot_status.get_mode() != RobotMode.DISABLED:
            if active_action is not None:
                if active_action.isFinished():
                    active_action.done()
                    active_action = None
                else:
                    active_action.update()
        else:
            active_action = None

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)