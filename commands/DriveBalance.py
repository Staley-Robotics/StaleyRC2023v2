# Import Python
import math

# Import FRC
from commands2 import Swerve4ControllerCommand
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians

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

class DriveBalance(Swerve4ControllerCommand):
    def __init__(self, swerveDrive:SwerveDrive):
        ### Trajectory Information
        # Trajectory Config
        config = TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelMetersPerSecondSq
        )

        trajectory = TrajectoryGenerator.generateTrajectory(
            start=Pose2d(Translation2d(0,0),Rotation2d(0)), # Start
            interiorWaypoints=[
                Translation2d(1,1),
                Translation2d(2,-1)
            ], # Path
            end=Pose2d(Translation2d(3,0),Rotation2d(0)), # End
            config=config # Config
        )

        ### PID Controllers
        pidX = PIDController(
            x_kP,
            x_kI,
            x_kD
        )
        pidY = PIDController(
            y_kP,
            y_kI,
            y_kD
        )
        pidT = ProfiledPIDControllerRadians(
            t_kP,
            t_kI,
            t_kD,
            TrapezoidProfileRadians.Constraints(
                kMaxAngularSpeedMetersPerSecond,
                kMaxSpeedMetersPerSecond
            )
        )
        
        super().__init__(
            trajectory=trajectory,
            pose=swerveDrive.getPose,
            kinematics=swerveDrive.kinematics,
            xController=pidX,
            yController=pidY,
            thetaController=pidT,
            desiredRotation=swerveDrive.getHeading,
            output=swerveDrive.runSwerveModuleStates,
            requirements=[swerveDrive]
        )
        self.setName( "DriveSCurve" )

