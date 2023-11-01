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


class TimeBasedMiddleHigh(SequentialCommandGroup):
    def __init__(self, swerveDrive: SwerveDrive, armPivot: ArmPivot, armExtend: ArmExtend, claw: Claw):
        super().__init__()
        self.setName("TimeBasedMiddleHigh")
        self.addRequirements([swerveDrive, armPivot, armExtend, claw])

        self.addCommands(ArmExtendReset(armExtend))
        self.addCommands(ClawClose(claw))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropTop))
        self.addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    DriveForTime(swerveDrive, -0.10, 0.0, 0.0, 0.3, False),
                    DriveForTime(swerveDrive, -0.25, 0.0, 0.0, 0.3, False),
                    DriveForTime(swerveDrive, -0.10, 0.0, 0.0, 0.3, True)
                ),
                ArmExtendPosition( armExtend, lambda: ArmExtendPosition.Position.maxValue )
            )
        )
        
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropTopRelease))
        self.addCommands(ClawOpen(claw))
        self.addCommands(ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.dropTop))
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.0, 0.0, 0.3, False))
        self.addCommands(DriveForTime(swerveDrive, 0.25, 0.0, 0.0, 0.3, False))
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.0, 0.0, 0.3, True))
        self.addCommands(ClawClose(claw))
        self.addCommands(
            SequentialCommandGroup(
                ArmExtendReset(armExtend),
                ArmPivotPosition(armPivot, lambda: ArmPivotPosition.Position.down)
            )
        )
        self.addCommands(DriveForTime(swerveDrive, 0.0, 0.0, 1.0, 0.25, True))
        self.addCommands(DriveForTime(swerveDrive, 1.0, 0.0, 0.0, 1.0, False))
        self.addCommands(DriveForTime(swerveDrive, 0.75, 0.0, 0.0, 0.4, False))
        self.addCommands(DriveForTime(swerveDrive, 0.50, 0.0, 0.0, 0.4, False)) 
        self.addCommands(DriveForTime(swerveDrive, 0.25, 0.0, 0.0, 0.4, False))  
        self.addCommands(DriveForTime(swerveDrive, 0.10, 0.0, 0.0, 0.4, True))