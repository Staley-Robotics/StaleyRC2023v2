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
class DriveByStick(CommandBase):
    def __init__( self,
                  swerveDrive:SwerveDrive,
                  velocityX:typing.Callable[[], float],
                  velocityY:typing.Callable[[], float],
                  holonomicX:typing.Callable[[], float] = ( lambda: 0.0 ),
                  holonomicY:typing.Callable[[], float] = ( lambda: 0.0 ), 
                  rotate:typing.Callable[[], float] = ( lambda: 0.0 )
                ):
        # CommandBase Initiation Configurations
        super().__init__()
        self.setName( "DriveByStick" )
        self.addRequirements( swerveDrive )
        
        # This Command Global Properties
        self.DriveSubsystem = swerveDrive
        self.vX = velocityX
        self.vY = velocityY
        self.hX = holonomicX
        self.hY = holonomicY
        self.rO = rotate

        # Slew Rate Limiters
        self.srl_vX = SlewRateLimiter( slrValue )
        self.srl_vY = SlewRateLimiter( slrValue )
        self.srl_hX = SlewRateLimiter( slrValue )
        self.srl_hY = SlewRateLimiter( slrValue )
        self.srl_rO = SlewRateLimiter( slrValue )

        self.hPid = self.DriveSubsystem.getHolonomicPIDController()
        self.tPid = self.hPid.getThetaController()

    def pidReset(self) -> None:
        self.tPid.reset( self.DriveSubsystem.getRobotAngle().radians() )

    def initialize(self) -> None:
        self.pidReset()

    def execute(self) -> None:
        # Get States
        halfSpeed = self.DriveSubsystem.isHalfspeed()

        # Get Input Values
        x = self.vX()
        y = self.vY()
        hX = self.hX()
        hY = self.hY()
        r = self.rO()

        # Calculate Deadband
        x = applyDeadband( x, driveDeadband ) 
        y = applyDeadband( y, driveDeadband )
        hX = applyDeadband( hX, driveDeadband )
        hY = applyDeadband( hY, driveDeadband )
        r = applyDeadband( r, driveDeadband )

        # Square the Inputs
        x *= abs( x )
        y *= abs( y )
        hX *= abs( hX )
        hY *= abs( hY )
        r *= abs( r )

        # Slew Rate Limiter
        x = self.srl_vX.calculate( x )
        y = self.srl_vY.calculate( y )
        #hX = self.srl_hX.calculate( hX )
        #hY = self.srl_hY.calculate( hY )
        r = self.srl_rO.calculate( r )

        # Calculate Half Speed
        magnitude:float = 1.0 if not halfSpeed else 0.5
        x *= magnitude
        y *= magnitude
        #r *= magnitude

        if abs(hX) > 0.1 or abs(hY) > 0.1:
            #pid = self.tPid
            mag = math.sqrt( hX*hX + hY*hY )
            robotAngle:float = self.DriveSubsystem.getRobotAngle().radians()
            goalAngle:float = Rotation2d( x=hX, y=hY ).radians()
            target = self.tPid.calculate(robotAngle, goalAngle)
            r = target * mag
            r = min( max( r, -1.0 ), 1.0 )
        elif abs(r) > 0.1:
            self.pidReset()

        # Send ChassisSpeeds
        self.DriveSubsystem.runPercentageInputs(x, y, r)

    def end(self, interrupted:bool) -> None: pass
    def isFinished(self) -> bool: return False
    def runsWhenDisabled(self) -> bool: return False