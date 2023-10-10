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
class ArmPivotByStick(CommandBase):
    def __init__( self,
                  armPivot:ArmPivot,
                  inputFunction:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"ArmPivotByStick" )
        
        self.armPivot = armPivot
        self.addRequirements( [self.armPivot] )

        self.input = inputFunction

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Get Input Values
        input = self.input()

        # Calculate Deadband
        input  = applyDeadband( input,   deadband ) 

        # Run Input
        if input != 0.0: 
            pos = self.armPivot.getPosition()
            pos += int(rate * input)
            self.armPivot.setPosition( pos )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False