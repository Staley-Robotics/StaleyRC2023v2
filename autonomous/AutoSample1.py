# Import Python
import math

# Import FRC
from commands2 import CommandBase, CommandGroupBase, Swerve4ControllerCommand, SequentialCommandGroup, ParallelCommandGroup
import commands2.cmd
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfile, TrapezoidProfileRadians

# Import Subsystems and Commands
from subsystems import *
from commands import ArmExtendByStick, ArmPivotByStick


class AutoSample1(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive, arm:ArmPivot, claw:Claw):
        super().__init__()
        self.setName( "AutoSample1" )
        self.addRequirements( [swerveDrive, arm, claw] )
        
        #self.addCommands( SCurve(swerveDrive) )
        self.addCommands( commands2.cmd.runOnce( swerveDrive.stop ) )
        self.addCommands( commands2.cmd.wait( 2.0 ) )
        #self.addCommands(
        #    ParallelCommandGroup(
        #        ArmExtendByStick(),
        #        ArmPivotByStick()
        #    )
        #)
        #self.addCommands( SCurve(swerveDrive) )
        self.addCommands( commands2.cmd.runOnce( swerveDrive.stop ) )
        self.addCommands( commands2.cmd.wait( 15.0 ) )


