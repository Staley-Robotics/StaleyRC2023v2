from commands2 import *
from subsystems import Navigation

class PovUpdate(CommandBase):
    def __init__(self, povSystem:Navigation, selector = None, position = None, action = None):
        super().__init__()
        self.PovSystem = povSystem

    def runsWhenDisabled(self) -> bool:
        return True