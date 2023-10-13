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
               
        values:list[int] = [
            0,
            10,
            100
        ]
        default:int = 10

    class Direction:
        kMax:int = 2
        kNext:int = 1
        kNone:int = 0
        kPrev:int = -1
        kMin:int = -2

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
        pos = self.Position.default
        
        match step:
            case self.Direction.kMax:
                max = len( self.Position.values ) -1
                pos = self.Position.values[max]
            case self.Direction.kNext:
                pass
            case self.Direction.kNone:
                pos = self.Position.default
            case self.Direction.kPrev:
                pass
            case self.Direction.kMin:
                pos = self.Position.values[0]
            case _:
                pos = self.Position.default   

        self.armPivot.setPosition( pos )

    def execute(self) -> None: pass
    def end(self, interrupted:bool) -> None: pass
    def isFinished(self) -> bool: return self.armPivot.atPosition()
    def runsWhenDisabled(self) -> bool: return False