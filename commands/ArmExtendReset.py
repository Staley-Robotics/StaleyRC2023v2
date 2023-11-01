### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpilib import Timer

# Our Imports
from subsystems import ArmExtend

### Constants
# SwerveDrive Module Input Deadband
resetPosition = -50000 # 50,000 Ticks
positionThreshold = 100 # Ticks
stallCycleThreshold = 5 # Number of Cycles to classify as a stall
timerExpiryThreshold = 2.0 # Time to determine stall

### DefaultArm Command Class
class ArmExtendReset(CommandBase):
    stallCycleCount = -1
    lastPosition = 0
    timer = Timer()

    def __init__(self, armExtend:ArmExtend):
        super().__init__()
        self.setName( f"ArmExtendReset" )
        self.addRequirements( armExtend )
        self.armExtend = armExtend

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()
        self.stallCycleCount = -1
        self.lastPosition = self.armExtend.getPosition()
        self.armExtend.setPosition( resetPosition, True )

    def execute(self) -> None: 
        currentPos = self.armExtend.getPosition()
        difference = currentPos - self.lastPosition
        if abs( difference ) < positionThreshold:
            self.stallCycleCount += 1
        else:
            self.stallCycleCount = 0
        self.lastPosition = currentPos
        self.armExtend.update()
    
    def end(self, interrupted:bool) -> None:
        self.timer.stop()
        #if not interrupted:
        self.armExtend.resetPosition()
        self.armExtend.update()

    def isFinished(self) -> bool:
        if self.timer.get() < timerExpiryThreshold:
            return self.stallCycleCount >= stallCycleThreshold
        else:
            return True
    
    def runsWhenDisabled(self) -> bool:
        return False
