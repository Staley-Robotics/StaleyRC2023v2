### Imports
# Python Imports
import typing
import math

# FRC Component Imports
from commands2 import CommandBase
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds

# Our Imports
from subsystems.swervedrive_2023 import SwerveDrive

### Constants
# SwerveDrive Module Inputs
driveDeadband = 0.04 # Deadband (Sensitivity towards Dead 0 on joystick)
slrValue = 2 # Slew Rate Limiter (Sensitivity to how fast the joysticks bounds back to 0)

# Default Drive Command Class
class DriveByStick(CommandBase):
    def __init__( self,
                  swerveDrive:SwerveDrive,
                  l_UpDown:typing.Callable[[], float],
                  l_LeftRight:typing.Callable[[], float],
                  r_UpDown:typing.Callable[[], float],
                  r_LeftRight:typing.Callable[[], float],
                  halfSpeed:typing.Callable[[], bool] = (lambda: False)
                ):
        # CommandBase Initiation Configurations
        super().__init__()
        self.setName( "DriveByStick" )
        self.addRequirements( swerveDrive )
        
        # This Command Global Properties
        self.DriveSubsystem = swerveDrive
        self.lx = l_UpDown
        self.ly = l_LeftRight
        self.rx = r_UpDown
        self.ry = r_LeftRight
        self.isHalfSpeed = halfSpeed

        # Slew Rate Limiters
        self.srl_l_ud = SlewRateLimiter( slrValue )
        self.srl_l_lr = SlewRateLimiter( slrValue )
        self.srl_r_ud = SlewRateLimiter( slrValue )
        self.srl_r_lr = SlewRateLimiter( slrValue )

    def execute(self) -> None:
        # Get States
        halfSpeed = self.isHalfSpeed()

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

        # Slew Rate Limiter
        x = self.srl_l_ud.calculate( x )
        y = self.srl_l_lr.calculate( y )
        r = self.srl_r_lr.calculate( r )

        # Calculate Half Speed
        magnitude:float = 1.0 if not halfSpeed else 0.5
        x *= magnitude
        y *= magnitude
        #r *= magnitude

        # Send ChassisSpeeds
        self.DriveSubsystem.runPercentageOutput(x, y, r)

    def initialize(self) -> None: pass
    def end(self, interrupted:bool) -> None: pass
    def isFinished(self) -> bool: return False
    def runsWhenDisabled(self) -> bool: return False