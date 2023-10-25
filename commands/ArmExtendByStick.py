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

### DefaultArm Command Class
class ArmExtendByStick(CommandBase):
    def __init__( self,
                  armExtend:ArmExtend,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmExtendByStick" )
        self.addRequirements( [armExtend] )

        self.__extend__ = armExtend
        self.__input__  = inputFunction

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        input = self.__input__()
        input = applyDeadband( input, deadband )
        self.__extend__.movePosition( input )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False