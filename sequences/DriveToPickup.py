# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup
import commands2.cmd
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

class DriveToPickup(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive):
        super().__init__( commands=() )
        self.setName( "Pickup" )
        self.addRequirements( swerveDrive )
        
        """
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
        """
        
        self.addCommands(
            ConditionalCommand(
                ConditionalCommand(
                    SequentialCommandGroup(
                        DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 2.175, 3.975), Rotation2d(0).fromDegrees(-180) ) ),
                        DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 2.925, 4.725), Rotation2d(0).fromDegrees(-180) ) )
                    ),
                    ConditionalCommand(
                        SequentialCommandGroup(
                            DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 2.175, 1.525), Rotation2d(0).fromDegrees(-180) ) ),
                            DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 2.925, 0.7625), Rotation2d(0).fromDegrees(-180) ) )
                        ),
                        commands2.cmd.nothing(),
                        lambda: True # Go Right
                    ),
                    lambda: False # Go Left
                ),
                commands2.cmd.nothing(),
                lambda: swerveDrive.getPose().X() < 2.925 and 1.525 < swerveDrive.getPose().Y() and swerveDrive.getPose().Y() < 3.975
            )
        )
        self.addCommands(
            ConditionalCommand(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 5.296, 4.725), Rotation2d(0).fromDegrees(179 ) ) ),
                    ConditionalCommand(
                        DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 5.296, 3.525), Rotation2d(0).fromDegrees(179 ) ) ),
                        ConditionalCommand(
                            DriveToPose( swerveDrive, lambda: Pose2d( Translation2d( 5.296, 0.7625), Rotation2d(0).fromDegrees(179 ) ) ),
                            commands2.cmd.nothing(),
                            lambda: -math.inf < swerveDrive.getPose().Y() and swerveDrive.getPose().Y() < 1.525 # Right
                        ),
                        lambda: 1.525 < swerveDrive.getPose().Y() and swerveDrive.getPose().Y() < 3.975  # Center
                    ),
                    lambda: 3.975 < swerveDrive.getPose().Y() and swerveDrive.getPose().Y() < 5.475  # Left
                ),
                commands2.cmd.nothing(),
                lambda: swerveDrive.getPose().X() < 5.296
            )
        )
        self.addCommands(
            ConditionalCommand(
                DriveToPose( swerveDrive, lambda: Pose2d( Translation2d(8.2615, 5.475), Rotation2d(0) ) ),
                commands2.cmd.nothing(),
                lambda: swerveDrive.getPose().X() < 12.750 and swerveDrive.getPose().Y() < 5.475
            )
        )
        self.addCommands(
            ConditionalCommand(
                DriveToPose( swerveDrive, lambda: Pose2d( Translation2d(12.750, 6.7750), Rotation2d(0) ) ),
                commands2.cmd.nothing(),
                lambda: swerveDrive.getPose().X() < 12.750
            )
        )
        self.addCommands(
            ConditionalCommand(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( Translation2d(15.576, 7.35), Rotation2d(0) ) ),
                    ConditionalCommand(
                        DriveToPose( swerveDrive, lambda: Pose2d( Translation2d(15.576, 6.20), Rotation2d(0) ) ),
                        commands2.cmd.nothing(),
                        lambda: True # Go Right
                    ),
                    lambda: False # Go Left
                ),
                commands2.cmd.nothing(),
                lambda: swerveDrive.getPose().X() < 15.576
            )
        )