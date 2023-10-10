"""
Description: 4 Wheel Swerve Drive Chassis
Version:  2
Date:  2023-10-04

Dependencies:
- SwerveModule:  SwerveModule_2023

CAN Network:  CANIVORE
Gyroscope:  PIGEON v2
Cameras:  limelight-one, limelight-two
"""

### Imports
# Python Imports
import typing
import math

# FRC Component Imports
from commands2 import SubsystemBase, CommandBase
from ctre.sensors import WPI_Pigeon2
from wpilib import SmartDashboard, Timer, DriverStation, RobotBase, SendableBuilderImpl
import wpiutil #import SendableBuilder
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.trajectory import Trajectory
from ntcore import NetworkTableInstance, NetworkTable

# Our Imports
from .swervemodule_2023_v2 import SwerveModule

### Constants
# SwerveDrive Module Input Deadband
driveDeadband = 0.04

# SwerveDrive Maximum Speeds
maxVelocity = 16

# Trajectory Maximums
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

### Class: SwerveDrive
class SwerveDrive(SubsystemBase):
    
    fieldRelative:bool = True
    halfSpeed:bool = False
    motionMagic:bool = False
    trajectory:Trajectory = None

    #def initSendable(self, sendable:wpiutil.SendableBuilder):
    #    super().initSendable(sendable)
        #    sendable.addBooleanProperty(
        #        "fieldRelative",
        #        self.getFieldRelative,
        #        self.setFieldRelative
        #    )
    #    pass

    def __init__(self):
        super().__init__()
        self.setSubsystem( "SwerveDrive" )
        self.setName( "SwerveDrive" )

        # Gyro
        self.gyro = WPI_Pigeon2( 61, "rio" )
        self.gyro.setYaw(-180.0)

        # Swerve Modules
        self.moduleFL = SwerveModule("FrontLeft",  7, 8, 18,  0.25,  0.25,  211.289)
        self.moduleFR = SwerveModule("FrontRight", 5, 6, 16,  0.25, -0.25,  125.068) #  35.684)
        self.moduleBL = SwerveModule("BackLeft",   3, 4, 14, -0.25,  0.25,  223.945)
        self.moduleBR = SwerveModule("BackRight",  1, 2, 12, -0.25, -0.25,   65.654)

        # Subsystem Dashboards
        self.addChild( "Gyro", self.gyro )
        #self.addChild( "FrontLeft", self.moduleFL )
        #self.addChild( "FrontRight", self.moduleFR )
        #self.addChild( "BackLeft", self.moduleBL )
        #self.addChild( "BackRight", self.moduleBR )

        # Kinematics
        self.kinematics = SwerveDrive4Kinematics(
            self.moduleFL.getModulePosition(),
            self.moduleFR.getModulePosition(),
            self.moduleBL.getModulePosition(),
            self.moduleBR.getModulePosition()
        )

        # Odometry
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            [
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleBL.getPosition(),
                self.moduleBR.getPosition()
            ],
            Pose2d(Translation2d(3,2), Rotation2d().fromDegrees(-180))
        )

        # Limelight
        self.limelight1 = NetworkTableInstance.getDefault().getTable( "limelight-one" )
        self.limelight2 = NetworkTableInstance.getDefault().getTable( "limelight-two" )


    ### Drive Based Functions
    # Stop Drivetrain
    def stop(self) -> CommandBase:
        return self.runOnce(
            lambda: self.runChassisSpeeds( ChassisSpeeds(0,0,0) )
        )

    # Returns Halfspeed Mode Status
    def isHalfspeed(self):
        return self.halfSpeed

    # Returns Field Relative Status
    def isFieldRelative(self):
        return self.fieldRelative


    ### ODOMETRY
    # Update Odometry Information on each loop
    def periodic(self):
        # Odometry from Limelight
        self.updateVisionOdometry( self.limelight1 )
        self.updateVisionOdometry( self.limelight2 )

        # Odometry from Module Position Data
        pose = self.odometry.updateWithTime(
            Timer.getFPGATimestamp(),
            self.gyro.getRotation2d(),
            [
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleBL.getPosition(),
                self.moduleBR.getPosition()
            ]
        )
        
        # Update Data on Dashboard
        SmartDashboard.putNumberArray(
            "Field/Swerve",
            [ pose.X(), pose.Y(), pose.rotation().degrees() ]
        )
    
    # Update Odometry From Vision Data
    def updateVisionOdometry(self, limelightData:NetworkTable):
        # Check for Valid Limelight Data
        aprilTag = limelightData.getNumber("tid",-1) 
        if aprilTag == -1: return
        
        # Check for Alliance Settings, Get Bot Pose Data from Limelight
        match DriverStation.getAlliance():
            case DriverStation.Alliance.kRed:
                botPose = limelightData.getNumberArray("botpose_wpired", None)
            case DriverStation.Alliance.kBlue:
                botPose = limelightData.getNumberArray("botpose_wpiblue", None)
            case DriverStation.Alliance.kInvalid:
                return None
            case _:
                return None

        # Translate Bot Pose Data to Pose2d
        llx = botPose[0]
        lly = botPose[1]
        llz = botPose[5]
        llt = botPose[6]
        llPose = Pose2d(llx, lly, Rotation2d(0).fromDegrees(llz))
        
        # Get Time Offset Information
        llOffset = Timer.getFPGATimestamp() - (llt/1000)
        
        # Update Odometry
        self.odometry.addVisionMeasurement(
            llPose,
            llOffset,
        )

    # Get Pose
    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()
    
    # Get Heading of Rotation
    def getHeading(self) -> Rotation2d:
        return self.gyro.getRotation2d()


    ### Run SwerveDrive Functions
    # Run SwerveDrive using ChassisSpeeds
    def runChassisSpeeds(self, speeds:ChassisSpeeds) -> typing.List[SwerveModuleState]:
        rotationCenter = Translation2d(0, 0)
        modStates = self.kinematics.toSwerveModuleStates(speeds, rotationCenter) # Convert to SwerveModuleState
        self.runSwerveModuleStates(list(modStates))

    # Run SwerveDrive using SwerveModuleStates
    noPrint = False
    def runSwerveModuleStates(self, states:typing.List[SwerveModuleState]) -> None:
        # Print Output in Simulation Environment
        if not RobotBase.isReal():
            if not self.noPrint: print( states )
            if states[0].speed != 0.0 and states[1].speed != 0.0 and states[2].speed != 0.0 and states[3].speed != 0.0:
                self.noPrint = False
            else:
                self.noPrint = True
    
        # Update Desired State for each Swerve Module
        modStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(states, maxVelocity)
        self.moduleFL.setDesiredState(modStates[0])
        self.moduleFR.setDesiredState(modStates[1])
        self.moduleBL.setDesiredState(modStates[2])
        self.moduleBR.setDesiredState(modStates[3])


    ### Field Relative Functions
    # Toggle Field Relative Property
    def fieldrelativeToggle(self) -> None:
        self.fieldRelative = not(self.fieldRelative)
        print( "Field Relative:", self.fieldRelative )

    # Set Field Relative: On
    def fieldrelativeOn(self) -> None:
        self.fieldRelative = True
        print( "Field Relative: On" )

    # Set Field Relative: Off
    def fieldrelativeOff(self) -> None:
        self.fieldRelative = False
        print( "Field Relative: Off" )

    def getFieldRelative(self) -> bool:
        return self.fieldRelative
    
    def setFieldRelative(self, fr:bool) -> None:
        self.fieldRelative = fr


    ### Half Speed Functions
    # Toggle Half Speed Property
    def halfspeedToggle(self) -> None:
        self.halfSpeed = not(self.halfSpeed)
        print( "Half Speed:", self.halfSpeed )

    # Set Half Speed: On
    def halfspeedOn(self) -> None:
        self.halfSpeed = True
        print( "Half Speed: On" )

    # Set Half Speed: Off
    def halfspeedOff(self) -> None:
        self.halfSpeed = False
        print( "Half Speed: Off" )


    ### Motion Magic Functions
    # Toggle Motion Magic Property
    def motionmagicToggle(self) -> None:
        self.motionMagic = not(self.motionMagic)
        print( "Motion Magic:", self.motionMagic )

    # Set Motion Magic: On
    def motionmagicOn(self) -> None:
        self.motionMagic = True
        self.motionmagicSetModule()
        print( "Motion Magic: On" )

    # Set Motion Magic: Off
    def motionmagicOff(self) -> None:
        self.motionMagic = False
        self.motionmagicSetModule()
        print( "Motion Magic: Off" )

    def motionmagicSetModule(self) -> None:
        self.moduleFL.setMotionMagic( self.motionMagic )
        self.moduleFR.setMotionMagic( self.motionMagic )
        self.moduleBL.setMotionMagic( self.motionMagic )
        self.moduleBR.setMotionMagic( self.motionMagic )

