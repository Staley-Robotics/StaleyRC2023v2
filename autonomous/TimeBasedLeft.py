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


class TimeBasedLeft(SequentialCommandGroup):
    def __init__(self, swerveDrive: SwerveDrive, armPivot: ArmPivot, armExtend: ArmExtend, claw: Claw):
        super().__init__()
        self.setName("LeftBasic")
        self.addRequirements([swerveDrive, armPivot, armExtend, claw])

        self.addCommands(ArmExtendReset(armExtend))
        self.addCommands(ClawClose(claw))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddle))
        self.addCommands(DriveForTime(swerveDrive, -0.10, 0.0, 0.0, 0.3, False))
        self.addCommands(DriveForTime(swerveDrive, -0.25, 0.0, 0.0, 0.3, False))
        self.addCommands(DriveForTime(swerveDrive, -0.10, 0.0, 0.0, 0.3, True))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddleRelease))
        self.addCommands(ClawOpen(claw))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropMiddle))
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.10, 0.0, 0.25, False))
        self.addCommands(DriveForTime(swerveDrive, 0.25, 0.25, 0.0, 1.50, False))
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.10, 0.0, 0.25, True))
        self.addCommands(ClawClose(claw))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.down))
        self.addCommands(DriveForTime(swerveDrive, 1.0, 0.0, 0.0, 1.0, False))
        self.addCommands(DriveForTime(swerveDrive, 0.75, 0.0, 0.0, 0.4, False))
        self.addCommands(DriveForTime(swerveDrive, 0.50, 0.0, 0.0, 0.4, False)) 
        self.addCommands(DriveForTime(swerveDrive, 0.25, 0.0, 0.0, 0.4, False))  
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.0, 0.0, 0.4, True))