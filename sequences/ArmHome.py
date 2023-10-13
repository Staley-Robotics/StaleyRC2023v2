# Import Python
import math

# Import FRC
from commands2 import ParallelCommandGroup

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants

# Class
class ArmHome(ParallelCommandGroup):
    def __init__(self, armPivot:ArmPivot, armExtend:ArmExtend, claw:Claw):
        super().__init__( commands=() )
        self.setName( "ArmHome" )
        self.addRequirements( [armPivot, armExtend, claw] )

        ### Command Sequence
        self.addCommands( ClawAction( claw, ClawAction.Action.kClose ) )
        self.addCommands( ArmExtendPosition( armExtend, lambda: ArmExtendPosition.Direction.kMin ) )
        self.addCommands( ArmPivotPosition( armPivot, lambda: ArmPivotPosition.Direction.kMin ) )