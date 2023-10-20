### Imports
# Python Imports
import typing
import math

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds

# Our Imports
from subsystems.SwerveDrive import SwerveDrive

### Constants
# SwerveDrive Module Inputs
driveDeadband = 0.04 # Deadband (Sensitivity towards Dead 0 on joystick)
slrValue = 2 # Slew Rate Limiter (Sensitivity to how fast the joysticks bounds back to 0)

# Default Drive Command Class
class DriveToRotation(CommandBase):
    def __init__( self,
                  swerveDrive:SwerveDrive,
                  degrees:typing.Callable[[], float] = ( lambda: 0.0 )
                ):
        # CommandBase Initiation Configurations
        super().__init__()
        self.setName( "DriveToRotation" )
        self.addRequirements( swerveDrive )
        
        # This Command Global Properties
        self.DriveSubsystem = swerveDrive
        self.hPid = self.DriveSubsystem.getHolonomicPIDController()
        self.tPid = self.hPid.getThetaController()
        self.rotation = Rotation2d(0).fromDegrees( degrees() )

    def initialize(self) -> None:
        self.tPid.reset( self.DriveSubsystem.getRobotAngle().radians() )

    def execute(self) -> None:
        pid = self.tPid
        mag = 1.0
        robotAngle:float = self.DriveSubsystem.getRobotAngle().radians()
        goalAngle:float = self.rotation.radians()
        target = pid.calculate(robotAngle, goalAngle)
        r = target * mag
        r = min( max( r, -1.0 ), 1.0 )

        # Send ChassisSpeeds
        self.DriveSubsystem.runPercentageInputs(0, 0, r)
  
    def end(self, interrupted:bool) -> None:
        self.DriveSubsystem.stop()
    def isFinished(self) -> bool:
        return self.tPid.atGoal()
    def runsWhenDisabled(self) -> bool: return False