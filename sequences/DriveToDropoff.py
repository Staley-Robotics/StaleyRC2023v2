# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

class DriveToDropoff(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive):
        super().__init__( commands=() )
        self.setName( "Dropoff" )
        self.addRequirements( swerveDrive )

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

        ### Command Sequence
        self.addCommands( DriveTrajectory( swerveDrive, trajectory ) )
        
