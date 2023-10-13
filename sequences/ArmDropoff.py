# Import Python
import math
import typing

# Import FRC
from commands2 import * #SequentialCommandGroup, ParallelCommandGroup

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants

# Class
class ArmDropOff(SequentialCommandGroup):
    def __init__( self,
                  armPivot:ArmPivot, 
                  armExtend:ArmExtend, 
                  claw:Claw, 
                  pivotPos:typing.Callable[[], float], 
                  extendPos:typing.Callable[[], float]
                ):
        super().__init__( commands=() )
        self.setName( "ArmHome" )
        self.addRequirements( [armPivot, armExtend, claw] )

        ### Command Sequence
        self.addCommands(
            ParallelCommandGroup(
                self.addCommands( ArmPivotPosition( armPivot, lambda: ArmPivotPosition.Direction.kMax ) ),
                SequentialCommandGroup(
                    WaitUntilCommand( lambda: armPivot.getPosition() > 5.0 ),
                    self.addCommands( ArmExtendPosition( armExtend, lambda: ArmExtendPosition.Direction.kMax ) )
                )
            )
        )
        self.addCommands( ArmPivotPosition( armPivot, lambda: ArmPivotPosition.Direction.kMax ) )
        self.addCommands(
            ParallelCommandGroup(
                ClawAction( claw, ClawAction.Action.kClose ),
                ArmExtendPosition( armExtend, lambda: ArmExtendPosition.Direction.kMin )
            )
        )
        self.addCommands( ArmPivotPosition( armPivot, lambda: ArmPivotPosition.Direction.kMin ) )