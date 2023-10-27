### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband

# Our Imports
from subsystems import ArmPivot

### Constants
# SwerveDrive Module Input Deadband
deadband = 0.04
slrValue = 2

### DefaultArm Command Class
class ArmPivotByStick(CommandBase):
    def __init__( self,
                  armPivot:ArmPivot,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmPivotByStick" )
        self.addRequirements( [armPivot] )
        
        self.__pivot__ = armPivot
        self.__input__ = inputFunction

        self.srl_vX = SlewRateLimiter( slrValue )


    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        input = self.__input__()
        input = applyDeadband( input,   deadband )
        input *= abs(input) 
        input = self.srl_vX.calculate( input )
        self.__pivot__.movePosition( input )
        
        self.__pivot__.update()

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False