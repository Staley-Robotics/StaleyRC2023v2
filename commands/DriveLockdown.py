# Import Python
import math

# Import FRC
from commands2 import CommandBase
from wpimath.kinematics import SwerveModuleState

# Import Subsystems and Commands
from subsystems import *

# Constants


class DriveLockdown(CommandBase):
    def __init__(self, swerveDrive:SwerveDrive):
        super().__init__()
        self.setName( "DriveLockdown" )

        self.swerveDrive = swerveDrive
        self.addRequirements( [self.swerveDrive] )

    def execute(self):
        modules = [
            SwerveModuleState( 0, Rotation2d(45) ),
            SwerveModuleState( 0, Rotation2d(135) ),
            SwerveModuleState( 0, Rotation2d(135) ),
            SwerveModuleState( 0, Rotation2d(45) )
        ]
        self.swerveDrive.runSwerveModuleStates( modules )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

