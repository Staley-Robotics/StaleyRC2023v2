# Import Python
import math
import typing

# Import FRC
from commands2 import CommandBase
from wpilib import Timer
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians
from pathplannerlib import PathConstraints, PathPlanner, PathPlannerTrajectory, PathPoint

# Import Subsystems and Commands
from subsystems import *


class DriveForTime(CommandBase):
    swerveDrive: SwerveDrive = None
    __timer__: Timer = Timer()

    def __init__(self, swerveDrive:SwerveDrive, x:float, y:float, r:float, time:float, stopAfter:bool) -> None:
        super().__init__()
        self.swerveDrive = swerveDrive
        self.addRequirements(swerveDrive)
        self.setName("DriveForTime")
        self.x = min(max(x, -1.0), 1.0)
        self.y = min(max(y, -1.0), 1.0)
        self.r = min(max(r, -1.0), 1.0)
        self.time = time
        self.stopAfter = stopAfter

    def initialize(self):
        self.__timer__.reset()
        self.__timer__.start()

    def execute(self):
        # Percentage Run
        self.swerveDrive.runPercentageInputs(self.x, self.y, self.r)

    def end(self, interrupted: bool) -> None:
        if self.stopAfter: self.swerveDrive.stop()
        self.__timer__.stop()

    def isFinished(self) -> bool:
        return not round( self.__timer__.get(), 2 ) < self.time

    def runsWhenDisabled(self) -> bool:
        return False
