### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband

# Our Imports
from subsystems import ArmExtend

### Constants
# SwerveDrive Module Input Deadband
deadband = 0.04
rate = 10

### DefaultArm Command Class
class ArmExtendPosition(CommandBase):
    class Position:
        minValue = 0.0
        maxValue = 16.0
        values:list[int] = [
            0,
            5,
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
                  armExtend:ArmExtend,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmExtendPosition" )
        self.addRequirements( armExtend )
        self.armExtend = armExtend

        self.input = inputFunction

    def initialize(self) -> None:
        step = self.input()
        #pos = self.Position.default
        
        #match step:
        #    case self.Direction.kMax:
        #        max = self.Position.values.count()
        #        pos = self.Position.values[max]
        #    case self.Direction.kNext:
        #       pass
        #    case self.Direction.kNone:
        #       pos = self.Position.default
        #    case self.Direction.kPrev:
        #        pass
        #    case self.Direction.kMin:
        #        pos = self.Position.values[0]
        #    case _:
        #        pos = self.Position.default   

        self.armExtend.setPositionInches( step )

    def execute(self) -> None: 
        self.armExtend.update()

    def end(self, interrupted:bool) -> None:
        self.armExtend.update()
        pass

    def isFinished(self) -> bool:
        return self.armExtend.atPosition()
    
    def runsWhenDisabled(self) -> bool:
        return False