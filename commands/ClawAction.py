### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpilib import Timer

# Our Imports
from subsystems.Claw import Claw


class ClawAction(CommandBase):
    class Action:
        kClose:int  = 0
        kOpen:int   = 1
        kToggle:int = -1

    def __init__(self, claw:Claw, action:int, noWait:bool=False):
        super().__init__()
        self.setName( "ClawAction" )
        self.addRequirements( claw )
        
        self.__claw__ = claw       
        self.__action__ = action
        self.__timer__ = Timer()
        self.__noWait__ = noWait


    def initialize(self) -> None:
        # Run Action
        match self.__action__:
            case ClawAction.Action.kClose:
                self.__claw__.close()
            case ClawAction.Action.kOpen:
                self.__claw__.open()
            case ClawAction.Action.kToggle:
                self.__claw__.toggle()
            case _:
                return
        
        # Start Timer
        self.__timer__.reset()
        self.__timer__.start()

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        self.__timer__.stop()

    def isFinished(self) -> bool:
        time = self.__timer__.get()
        return self.__noWait__ or time > 0.4
    
    def runsWhenDisabled(self) -> bool:
        return False

class ClawOpen(ClawAction):
    def __init__(self, claw, noWait:bool=False):
        super().__init__(claw, ClawAction.Action.kOpen, noWait)

class ClawClose(ClawAction):
    def __init__(self, claw, noWait:bool=False):
        super().__init__(claw, ClawAction.Action.kClose, noWait)

class ClawToggle(ClawAction):
    def __init__(self, claw, noWait:bool=False):
        super().__init__(claw, ClawAction.Action.kToggle, noWait)