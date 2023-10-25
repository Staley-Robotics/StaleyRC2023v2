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


class RightBasic(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive, armPivot:ArmPivot, armExtend:ArmExtend, claw:Claw, redAlliance:bool = False):
        super().__init__()
        self.setName( "RightBasic" )
        #self.addRequirements( [swerveDrive, armPivot, armExtend, claw] )
        
        StartPose = ChargedUp.getPose( "DropoffRobotMiddle", "Dropoff8", 180, redAlliance )
        PlacePose = ChargedUp.getPose( "DropoffRobotEdge", "Dropoff8", 180, redAlliance )
        StepOutPose = ChargedUp.getPose( "DropoffRobotMiddle", "SafeExitRight", 180, redAlliance )
        ExitPose = ChargedUp.getPose( "SafeExit", "SafeExitRight", 180, redAlliance )

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



