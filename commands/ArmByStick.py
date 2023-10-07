### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband

# Our Imports
from subsystems.arm import Arm, ArmPivot, ArmExtend

### Constants
# SwerveDrive Module Input Deadband
driveDeadband = 0.04
rate = 10
pivotRate = 10
extendRate = 10

### DefaultArm Command Class
class ArmByStick(CommandBase):
    def __init__( self,
                  arm:Arm,
#                  pivot:typing.Callable[[], float],
#                  extend:typing.Callable[[], float]
                  input:typing.Callable[[], float]
                ):
        super().__init__()
        self.setName( f"{type(arm)}ByStick" )
        
        self.arm = arm
        #self.ArmPivot = armPivot
        #self.ArmExtend = armExtend
        self.addRequirements( [self.arm] )
        #self.addRequirements( [self.armPivot, self.ArmExtend] )

        self.input = input
        #self.pivot = pivot
        #self.extend = extend


    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Get Input Values
        input = self.input()
        #pivot = self.pivot()
        #extend = self.extend()

        # Calculate Deadband
        #pivot  = applyDeadband( pivot,   driveDeadband ) 
        #extend = applyDeadband( extend,   driveDeadband )

        # Run Input
        if input != 0.0: 
            pos = self.arm.getPosition()
            pos += int(rate * input)
            self.arm.setPosition( pos )


        # Run Pivot        
        #if pivot != 0.0: 
        #    pivotPos = self.ArmPivot.getPosition()
        #    pivotPos += int(pivotRate * pivot)
        #    self.ArmExtend.setPosition( pivotPos )

        # Run Extend
        #if extend != 0.0: 
        #    extendPos = self.ArmExtend.getPosition()
        #    extendPos += int(extendRate * extend)
        #    self.ArmExtend.getPosition( extendPos )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False