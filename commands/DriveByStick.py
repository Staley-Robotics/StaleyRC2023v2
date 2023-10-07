### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband
from wpimath.kinematics import ChassisSpeeds

# Our Imports
from subsystems.swervedrive_2023 import SwerveDrive

### Constants
# SwerveDrive Module Input Deadband
driveDeadband = 0.04

# SwerveDrive Maximum Speeds
maxVelocity = 16
maxAngularVelocity = 100.48

# Default Drive Command Class
class DriveByStick(CommandBase):
    def __init__( self,
                  swerveDrive:SwerveDrive,
                  l_UpDown:typing.Callable[[], float],
                  l_LeftRight:typing.Callable[[], float],
                  r_UpDown:typing.Callable[[], float],
                  r_LeftRight:typing.Callable[[], float],
                  halfSpeed:typing.Callable[[], bool] = (lambda: False),
                  fieldRelative:typing.Callable[[], bool] = (lambda: True)
                ):
        super().__init__()
        self.setName( "DriveByStick" )
        
        self.DriveSubsystem = swerveDrive
        self.addRequirements( [self.DriveSubsystem] )

        self.lx = l_UpDown
        self.ly = l_LeftRight
        self.rx = r_UpDown
        self.ry = r_LeftRight
        self.isHalfSpeed = halfSpeed
        self.isFieldRelative = fieldRelative

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Get States
        halfSpeed = self.isHalfSpeed()
        fieldRelative = self.isFieldRelative()

        # Get Input Values
        x = self.lx()
        y = self.ly()
        r = self.ry()

        # Square the Inputs
        x *= abs( x )
        y *= abs( y )
        r *= abs( r )

        # Calculate Deadband
        x = applyDeadband( x, driveDeadband ) 
        y = applyDeadband( y, driveDeadband )
        r = applyDeadband( r, driveDeadband )

        # Calculate Half Speed
        magnitude:float = 1.0 if not halfSpeed else 0.5
        x *= magnitude
        y *= magnitude
        r *= magnitude

        # Drive        
        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx = x * maxVelocity,
                vy = y * maxVelocity,
                omega = r * maxAngularVelocity,
                robotAngle = self.DriveSubsystem.getHeading()
            )
        else:
            speeds = ChassisSpeeds(
                vx = x * maxVelocity,
                vy = y * maxVelocity,
                omega = r * maxAngularVelocity
            )

        # Send ChassisSpeeds
        self.DriveSubsystem.runChassisSpeeds(speeds)


    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return False