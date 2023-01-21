from hmi_agent_node.default_actions.Action import Action
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status
import rospy
from frc_robot_utilities_py_node.SubsystemController import SubsystemController
from ck_utilities_py_node.ckmath import *
from typing import List
from hmi_agent_node.game_specific_actions.Subsystem import Subsystem

class MoveArmAction(Action):
    arm_subsystem = SubsystemController[Arm_Control, Arm_Status]('ArmControl', Arm_Control, 'ArmStatus', Arm_Status)

    def __init__(self, arm_base_position : float, arm_upper_position : float, position_delta_threshold : float = 0.1):
        self.__Arm_Control_msg = Arm_Control()
        self.__Arm_Control_msg.arm_base_requested_position = arm_base_position
        self.__Arm_Control_msg.arm_upper_requested_position = arm_upper_position
        self.__position_delta_threshold = position_delta_threshold

    def start(self):
        self.arm_subsystem.publish(self.__Arm_Control_msg)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        if self.arm_subsystem.get() is None:
            rospy.logerr("No status update present from arm")
            return False

        return within(self.arm_subsystem.get().arm_base_actual_position, self.__Arm_Control_msg.arm_base_requested_position, self.__position_delta_threshold) and \
            within(self.arm_subsystem.get().arm_upper_actual_position, self.__Arm_Control_msg.arm_upper_requested_position, self.__position_delta_threshold)

    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.ARM }