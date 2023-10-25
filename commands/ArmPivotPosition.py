### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband

# Our Imports
from subsystems import ArmPivot

### Constants
# SwerveDrive Module Input Deadband
deadband = 0.04
rate = 10

### DefaultArm Command Class
class ArmPivotPosition(CommandBase):
    class Position:
        pickup:int = 90.0
        pickupTakeoff:int = 91.0

        dropTop:int = 90.0
        dropTopRelease:int = 88.0

        dropMiddle:int = 10.0
        dropMiddleRelease:int = 9.0

        dropBottom:int = 5.0
        dropBottomRelease:int = 5.0

        down:int = 1.0
               
        values:list[int] = [
            0,
            10,
            100
        ]
        default:int = 10

    def __init__( self,
                  armPivot:ArmPivot,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmPivotPosition" )
        self.addRequirements( armPivot )
        self.armPivot = armPivot

        self.input = inputFunction

    def initialize(self) -> None:
        step = self.input()
        self.armPivot.setPosition( step )

    def execute(self) -> None: pass
    def end(self, interrupted:bool) -> None: pass
    def isFinished(self) -> bool: return self.armPivot.atPosition()
    def runsWhenDisabled(self) -> bool: return False