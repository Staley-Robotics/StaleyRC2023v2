# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants

# Class
class ArmHome(SequentialCommandGroup):
    def __init__(self, armPivot:ArmPivot, armExtend:ArmExtend, claw:Claw):
        super().__init__( commands=() )
        self.setName( "ArmHome" )
        self.addRequirements( [armPivot, armExtend, claw] )

        ### Command Sequence
        self.addCommands( ArmPivotPosition( armPivot, lambda: ArmPivotPosition.Direction.kMax ) )
        self.addCommands( ClawAction( claw, ClawAction.Action.kOpen ) )
