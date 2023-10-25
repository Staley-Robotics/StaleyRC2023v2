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
from commands import *


class LeftBasic(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive, armPivot:ArmPivot, armExtend:ArmExtend, claw:Claw, redAlliance:bool = False):
        super().__init__()
        self.setName( "LeftBasic" )
        #self.addRequirements( [swerveDrive, armPivot, armExtend, claw] )
        
        StartPose = ChargedUp.getPose( "DropoffRobotMiddle", "Dropoff2", 180, redAlliance )
        PlacePose = ChargedUp.getPose( "DropoffRobotEdge", "Dropoff2", 180, redAlliance )
        StepOutPose = ChargedUp.getPose( "DropoffRobotMiddle", "SafeExitLeft", 180, redAlliance )
        ExitPose = ChargedUp.getPose( "SafeExit", "SafeExitLeft", 180, redAlliance )

        self.addCommands(
            ParallelCommandGroup(
                DriveToPose(swerveDrive, lambda: StartPose),
                ArmExtendReset( armExtend ),
                ClawClose(claw)
            )
        )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddle))
        self.addCommands( DriveToPose(swerveDrive, lambda: PlacePose) )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddleRelease))
        self.addCommands( ClawOpen(claw) )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddle))
        self.addCommands(
            ParallelCommandGroup(
                DriveToPose(swerveDrive, lambda: StepOutPose),
                ClawClose(claw)
            ) 
        )
        self.addCommands( ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.down))
        self.addCommands( DriveToPose(swerveDrive, lambda: ExitPose) )



