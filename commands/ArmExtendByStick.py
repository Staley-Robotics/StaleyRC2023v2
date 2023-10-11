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
pivotRate = 10
extendRate = 10

### DefaultArm Command Class
class ArmExtendByStick(CommandBase):
    def __init__( self,
                  armExtend:ArmExtend,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmExtendByStick" )
        
        self.armExtend = armExtend
        self.addRequirements( [self.armExtend] )

        self.input = inputFunction

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Get Input Values
        input = self.input()

        # Calculate Deadband
        input = applyDeadband( input, deadband )

        # Run Input
        #if input != 0.0: 
        #    pos = self.armExtend.getPosition()
        #    pos += int(rate * input)
        #    self.armExtend.setPosition( pos )

        self.armExtend.movePosition( input )


    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False