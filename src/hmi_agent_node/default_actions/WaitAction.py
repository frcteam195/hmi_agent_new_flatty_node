from hmi_agent_node.default_actions.Action import Action
from datetime import datetime
from typing import List
from hmi_agent_node.game_specific_actions.Subsystem import Subsystem

class WaitAction(Action):
    def __init__(self, time_to_wait : float):
        self.__time_to_wait = time_to_wait
        self.__start_time = datetime.now()

    def start(self):
        self.__start_time = datetime.now()

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        duration = datetime.now() - self.__start_time
        return duration.total_seconds() > self.__time_to_wait

    def affectedSystems(self) -> List[Subsystem]:
        return { Subsystem.NONE }