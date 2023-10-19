# Import Python
import math

# Import FRC
from commands2 import CommandBase
from wpilib import Timer
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians

# Import Subsystems and Commands
from subsystems import *

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

x_kP = 1
x_kI = 0
x_kD = 0

y_kP = 1
y_kI = 0
y_kD = 0

t_kP = 1
t_kI = 0
t_kD = 0

class DriveToPosition(CommandBase):
    _m_timer = Timer()
    _m_kinematics: SwerveDrive4Kinematics = None
    _m_controller: HolonomicDriveController = None
    _m_trajectory: Trajectory = None

    swerveDrive: SwerveDrive = None
    
    def __init__(self, swerveDrive:SwerveDrive, trajectoryCmd:typing.Callable[[], Trajectory]):
        super().__init__()
        ### PID Controllers
        pidX = PIDController( x_kP, x_kI, x_kD )
        pidY = PIDController( y_kP, y_kI, y_kD )
        pidT = ProfiledPIDControllerRadians( t_kP, t_kI, t_kD,
            TrapezoidProfileRadians.Constraints(
                kMaxAngularSpeedMetersPerSecond,
                kMaxAngularAccelMetersPerSecondSq
            )
        )
        self._m_controller = HolonomicDriveController( pidX, pidY, pidT )
        
        self.swerveDrive = swerveDrive
        self.trajectoryCmd = trajectoryCmd
        self.addRequirements( swerveDrive )
        self.setName( "DriveToPosition" )

    def initialize(self):
        self._m_trajectory = self.trajectoryCmd()
        if self._m_trajectory != None:
            self._m_timer.restart()
        else:
            print( "Cancelled!" )
            self.cancel()

    def execute(self):
        if self._m_trajectory == None: return
        
        curTime = self._m_timer.get()
        desiredState = self._m_trajectory.sample(curTime)
        targetChassisSpeeds = self._m_controller.calculate()  (
            self.swerveDrive.getPose(),
            desiredState,
            self.swerveDrive.getHeading()
        )
        self.swerveDrive.runChassisSpeeds( targetChassisSpeeds )

    def end(self, interrupted:bool) -> None:
        self._m_timer.stop()

    def isFinished(self) -> bool:
        returnMe = False

        if self._m_trajectory != None:
            returnMe = self._m_timer.hasElapsed( self._m_trajectory.totalTime() )
        
        return returnMe
        
