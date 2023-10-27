from commands2 import *
from wpilib import *
from wpimath.geometry import *
from subsystems import *

tolerance = 0.5

class DriveBalance(CommandBase):
    def __init__(self, DriveTrain:SwerveDrive) -> None:
        # Command Base
        super().__init__()
        self.addRequirements( DriveTrain )
        self.setName( "Balance" )

        ### PID Controllers
        self._m_controller = DriveTrain.getHolonomicPIDController()
        self._m_xPid = self._m_controller.getXController()
        self._m_yPid = self._m_controller.getYController()
        self._m_tPid = self._m_controller.getThetaController()

        # Globalize
        self.__DriveTrain__ = DriveTrain

    def initialize(self) -> None:
        # Pid Controller Resets
        self._m_xPid.reset()
        self._m_yPid.reset()
        self._m_tPid.reset( self.__DriveTrain__.getRobotAngle().radians() )
    
    def execute(self) -> None:
        pass
        #targetPose = self.__DriveTrain__.getRobotAngle().radians()
        #
        #x, y, z = self.__DriveTrain__.getGyroGravityVector()
        #angle = Rotation2d( x, y ).degrees()
        #vector = math.sqrt( (x*x) + (y*y) )
        #if angle < 0.0: vector *= -1
        #
        #x = self._m_xPid.calculate( vector, 0 )
        #y = self._m_yPid.calculate( 0, 0 )
        #r = self._m_tPid.calculate( targetPose, targetPose )

        #self.__DriveTrain__.runPercentageInputs( x, y, r )
    
    def end(self, interrupted: bool) -> None:
        self.__DriveTrain__.stop()
    
    def isFinished(self) -> bool:
        x, y, z = self.__DriveTrain__.getGyroGravityVector()
        return abs( x ) < tolerance and abs( y ) < tolerance
    
    def runsWhenDisabled(self) -> bool:
        False