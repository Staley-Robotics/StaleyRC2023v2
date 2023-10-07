# Import Python
import math

# Import FRC
from commands2 import Swerve4ControllerCommand
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
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

class DriveTrajectory(Swerve4ControllerCommand):
    def __init__(self, swerveDrive:SwerveDrive, trajectory:Trajectory):
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
                kMaxAngularAccelMetersPerSecondSq
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
        self.setName( "DrivePath" )
