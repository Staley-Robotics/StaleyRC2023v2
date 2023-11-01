# Import Python
import math

# Import FRC
from commands2 import CommandBase, CommandGroupBase, Swerve4ControllerCommand, SequentialCommandGroup, ParallelCommandGroup
import commands2.cmd

# Import Subsystems and Commands
from subsystems import *
from commands import *


class ArmFluid(SequentialCommandGroup):
    def __init__(self, armPivot:ArmPivot, armExtend:ArmExtend, claw:Claw):
        super().__init__()
        self.setName( "ArmFluid" )
        self.addRequirements( armPivot, armExtend, claw)

        self.addCommands( ClawAction(claw, ClawAction.Action.kToggle) )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropTop) )
        self.addCommands( 
            ParallelCommandGroup(
                ArmExtendReset(armExtend),
                ClawAction(claw, ClawAction.Action.kClose, True)
            )
        )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.down) )