# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup, CommandBase
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
import commands2.cmd

# Import Subsystems and Commands
from subsystems import *
from commands import *
#from commands.DriveTrajectory import DriveTrajectory

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = 2 * math.pi
kMaxAngularAccelMetersPerSecondSq = 2 * math.pi

class SCurve(SequentialCommandGroup):
    m_trajectory:Trajectory = None

    def __init__(self, swerveDrive:SwerveDrive):
        super().__init__( commands=() )
        self.setName( "SCurve" )
        self.addRequirements( swerveDrive )
        self.swerveDrive = swerveDrive

        ### Command Sequence
        self.addCommands( commands2.cmd.runOnce( lambda: self.buildTrajectory() ) )
        self.addCommands( DriveTrajectory( swerveDrive, lambda: self.getTrajectory() ) )

    def end(self, interupted:bool):
        self.m_trajectory = None


    ### Trajectory Commands
    def buildTrajectory(self):
        startPose = self.swerveDrive.getPose()
        x = startPose.X()
        y = startPose.Y()
        waypoints = [
            Translation2d( x+1, y+0.5 ),
            Translation2d( x+2, y-0.5 )
        ]
        endPose = Pose2d(
            Translation2d( x+3, y ),
            startPose.rotation()
        )
        config = TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelMetersPerSecondSq
        )

        self.m_trajectory = TrajectoryGenerator.generateTrajectory(
            start=startPose, # Start
            interiorWaypoints=waypoints, # Path
            end=endPose, # End
            config=config # Config
        )

    def getTrajectory(self) -> Trajectory:
        return self.m_trajectory

