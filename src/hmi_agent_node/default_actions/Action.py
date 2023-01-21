from abc import ABC, abstractmethod
from typing import List
from hmi_agent_node.game_specific_actions.Subsystem import Subsystem

class Action(ABC):
    @abstractmethod
    def isFinished(self) -> bool:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def done(self):
        pass

    @abstractmethod
    def affectedSystems(self) -> List[Subsystem]:
        pass 