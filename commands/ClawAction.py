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

    def __init__(self, claw:Claw, action:int):
        super().__init__()
        self.setName( "ClawAction" )
        
        self.ClawSubsystem = claw
        self.addRequirements( [self.ClawSubsystem] )
               
        self.action = action

        self.timer = Timer()


    def initialize(self) -> None:
        # Run Action
        match self.action:
            case ClawAction.Action.kClose:
                self.ClawSubsystem.close()
            case ClawAction.Action.kOpen:
                self.ClawSubsystem.open()
            case ClawAction.Action.kToggle:
                self.ClawSubsystem.toggle()
            case _:
                return
        
        # Start Timer
        self.timer.start()

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        self.timer.stop()
        self.timer.reset()

    def isFinished(self) -> bool:
        time = self.timer.get()
        return time > 0.4
    
    def runsWhenDisabled(self) -> bool:
        return False

