# Import Python
import math

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

kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi
maxAngularVelocity = 2 * math.pi

class DriveToPose(CommandBase):
    _m_controller: HolonomicDriveController = None
    swerveDrive: SwerveDrive = None
    
    def __init__(self, swerveDrive:SwerveDrive, getTargetPose:typing.Callable[[], Pose2d]):
        super().__init__()
        ### PID Controllers
        self._m_controller = swerveDrive.getHolonomicPIDController()
        self._m_xPid = self._m_controller.getXController()
        self._m_yPid = self._m_controller.getYController()
        self._m_tPid = self._m_controller.getThetaController()
        
        self.swerveDrive = swerveDrive
        self.getTargetPose = getTargetPose
        self.addRequirements( swerveDrive )
        self.setName( "DriveToPose" )

    def initialize(self):
        # Pid Controller Resets
        self._m_xPid.reset()
        self._m_yPid.reset()
        self._m_tPid.reset( self.swerveDrive.getRobotAngle().radians() )

    def execute(self):
        startPose = self.swerveDrive.getPose()
        targetPose = self.getTargetPose()

        x = self._m_xPid.calculate( startPose.X(), targetPose.X() )
        y = self._m_yPid.calculate( startPose.Y(), targetPose.Y() )
        r = self._m_tPid.calculate( startPose.rotation().radians(), targetPose.rotation().radians() )

        x = min( max( x, -1.0 ), 1.0 )
        y = min( max( y, -1.0 ), 1.0 )
        r = min( max( r, -1.0 ), 1.0 )

        print( f"{round(x,3)}, {round(y,3)}, {round(r,3)}")
        self.swerveDrive.runPercentageInputs( x, y, r )
        #speeds = ChassisSpeeds.fromFieldRelativeSpeeds( x, y, r, self.swerveDrive.getRobotAngle() )
        #self.swerveDrive.runChassisSpeeds( speeds )

    def end(self, interrupted:bool) -> None:
        pass #self.swerveDrive.stop()

    def isFinished(self) -> bool:
        xDone = self._m_xPid.atSetpoint()
        yDone = self._m_yPid.atSetpoint()
        tDone = self._m_tPid.atSetpoint()
        return xDone and yDone and tDone