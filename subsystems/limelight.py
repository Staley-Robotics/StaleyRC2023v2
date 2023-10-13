"""
Description: Limelight Subsystem
Version:  1
Date:  2023-10-06

Dependencies:
- None
"""

### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import SubsystemBase
from wpilib import Timer, DriverStation
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from ntcore import NetworkTableInstance, NetworkTable

# Our Imports

### Constants

### Class: SwerveDrive
class Limelight(SubsystemBase):
    def __init__(self, name:str, odometryFunction:typing.Callable[[], SwerveDrive4PoseEstimator]):
        super().__init__()
        self.table:NetworkTable = NetworkTableInstance.getDefault().getTable( name )
        self.getOdometry = odometryFunction

    def periodic(self):
        # Check for Valid Limelight Data
        aprilTag = self.table.getNumber("tid",-1) 
        if int(aprilTag) == -1: return
        
        # Check for Alliance Settings, Get Bot Pose Data from Limelight
        match DriverStation.getAlliance():
            case DriverStation.Alliance.kRed:
                botPose = self.table.getNumberArray("botpose_wpired", None)
            case DriverStation.Alliance.kBlue:
                botPose = self.table.getNumberArray("botpose_wpiblue", None)
            case DriverStation.Alliance.kInvalid:
                return None
            case _:
                return None

        # Translate Bot Pose Data to Pose2d
        if botPose == None: return
        llx = botPose[0]
        lly = botPose[1]
        llz = botPose[5]
        llt = botPose[6]
        llPose = Pose2d(llx, lly, Rotation2d(0).fromDegrees(llz))
        
        # Get Time Offset Information
        llOffset = Timer.getFPGATimestamp() - (llt/1000)
        
        # Update Odometry
        self.getOdometry().addVisionMeasurement(
            llPose,
            llOffset
        )