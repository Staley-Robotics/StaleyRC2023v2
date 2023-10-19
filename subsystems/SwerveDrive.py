"""
Description: 4 Wheel Swerve Drive Chassis
Version:  2
Date:  2023-10-04

Dependencies:
- SwerveModule:  SwerveModule_2023

CAN Network:  CANIVORE
Gyroscope:  PIGEON v2
"""

### Imports
# Python Imports
import typing
import math

# FRC Component Imports
from commands2 import SubsystemBase, CommandBase
from ctre.sensors import WPI_Pigeon2
from wpilib import SmartDashboard, Timer, DriverStation, RobotBase, SendableBuilderImpl, Field2d, FieldObject2d
from wpiutil import SendableBuilder
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from ntcore import NetworkTableInstance, NetworkTable

# Our Imports
from .SwerveModule import SwerveModule

### Constants
# SwerveDrive Module Input Deadband
gyroStartHeading = -180.0

# SwerveDrive Maximum Speeds
maxVelocity = 3.75
maxAngularVelocity = 2 * math.pi

# Trajectory Maximums
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi
maxAngularVelocity = 2 * math.pi

### Class: SwerveDrive
class SwerveDrive(SubsystemBase):
    class BooleanProperty():
        _name:str = ""
        _value:bool = False
        def __init__(self, name:str, value:bool = False) -> None:
            self._name = name
            self.set( value )
        def set(self, value:bool) -> None:
            self._value = value
            SmartDashboard.putBoolean( self._name, value )
        def get(self) -> bool:
            return self._value
        def toggle(self) -> None:
            current = self.get()
            self.set(not current)
    
    fieldRelative:BooleanProperty = BooleanProperty("fieldRelative", True)
    halfSpeed:BooleanProperty = BooleanProperty("halfSpeed", False)
    motionMagic:BooleanProperty = BooleanProperty("motionMagic", False)
    trajectory:Trajectory = None
    holonomicPID:HolonomicDriveController = None

    def __init__(self):
        super().__init__()
        self.setSubsystem( "SwerveDrive" )
        self.setName( "SwerveDrive" )

        # Gyro
        self.gyro = WPI_Pigeon2( 61, "rio" )
        self.gyro.setYaw( gyroStartHeading )
        if RobotBase.isSimulation(): self.gyro.getSimCollection().setRawHeading( gyroStartHeading )

        # Swerve Modules
        self.moduleFL = SwerveModule("FrontLeft",  7, 8, 18,  0.25,  0.25,   31.289 ) #211.289)
        self.moduleFR = SwerveModule("FrontRight", 5, 6, 16,  0.25, -0.25,  -54.932 ) #125.068) #  35.684)
        self.moduleBL = SwerveModule("BackLeft",   3, 4, 14, -0.25,  0.25,   43.945 ) #223.945)
        self.moduleBR = SwerveModule("BackRight",  1, 2, 12, -0.25, -0.25, -114.346 )  #65.654)

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
            Pose2d(Translation2d(3,2), Rotation2d().fromDegrees(gyroStartHeading))
        )

        # Holonomic PID
        constraints = TrapezoidProfileRadians.Constraints(
            kMaxAngularSpeedMetersPerSecond,
            kMaxAngularAccelMetersPerSecondSq
        )
        xPid = PIDController( 1, 0, 0 )
        xPid.reset()
        yPid = PIDController( 1, 0, 0 )
        yPid.reset()
        tPid = ProfiledPIDControllerRadians( 1, 0, 0, constraints )
        tPid.enableContinuousInput( -math.pi, math.pi )
        tPid.reset(0)
        self.holonomicPID = HolonomicDriveController(
            xPid,
            yPid,
            tPid
        )
        self.holonomicPID.setTolerance(
            Pose2d(
                Translation2d( 0.025, 0.025 ),
                Rotation2d(0).fromDegrees( 0.025 )
            )
        )

        # Field on Shuffleboard
        SmartDashboard.putData("Field", Field2d())

    ### Drive Based Functions
    # Stop Drivetrain
    def stop(self) -> CommandBase:
        return self.runOnce(
            lambda: self.runChassisSpeeds( ChassisSpeeds(0,0,0) )
        )

    # Returns Halfspeed Mode Status
    def isHalfspeed(self):
        return self.halfSpeed.get()

    # Returns Field Relative Status
    def isFieldRelative(self):
        return self.fieldRelative.get()

    ### PID Controller
    def getHolonomicPIDController(self) -> HolonomicDriveController:
        return self.holonomicPID

    ### ODOMETRY
    # Update Odometry Information on each loop
    def periodic(self):
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
        poseX = round( pose.X(), 3 )
        poseY = round( pose.Y(), 3 )
        poseR = round( pose.rotation().degrees(), 3 )
        SmartDashboard.putNumberArray(
            "Field/RealRobot",
            [ poseX, poseY, poseR ]
        )
       
    # Resync the Gyro
    def syncGyro(self) -> None:
        poseDegrees = self.getPose().rotation().degrees()
        gyroDegrees = self.gyro.getAngle()
        degreesDiff = gyroDegrees - poseDegrees
        if ( abs(degreesDiff) < 0.1 ):
            pose = self.getPose()
            self.gyro.setYaw( pose.rotation().degrees() )
            print( f"Gyro Off by: {degreesDiff} Degrees | Resetting Gryo to {poseDegrees}")

    # Get Odometry Object
    def getOdometry(self) -> SwerveDrive4PoseEstimator:
        return self.odometry

    # Get Pose
    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()
    
    # Get Heading of Rotation
    def getRobotAngle(self) -> Rotation2d:
        return self.gyro.getRotation2d()

    ### Run SwerveDrive Functions
    def runPercentageInputs(self, x:float = 0.0, y:float = 0.0, r:float = 0.0) -> ChassisSpeeds:
        if self.fieldRelative.get():
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx = x * maxVelocity,
                vy = y * maxVelocity,
                omega = r * maxAngularVelocity,
                robotAngle = self.getRobotAngle()
            )
        else:
            speeds = ChassisSpeeds(
                vx = x * maxVelocity,
                vy = y * maxVelocity,
                omega = r * maxAngularVelocity
            )

        # Send ChassisSpeeds
        self.runChassisSpeeds(speeds)

    # Run SwerveDrive using ChassisSpeeds
    def runChassisSpeeds(self, speeds:ChassisSpeeds) -> None:
        rotationCenter = Translation2d(0, 0)
        modStates = self.kinematics.toSwerveModuleStates(speeds, rotationCenter) # Convert to SwerveModuleState
        self.runSwerveModuleStates(list(modStates))

    # Run SwerveDrive using SwerveModuleStates
    noPrint = False
    def runSwerveModuleStates(self, states:typing.List[SwerveModuleState]) -> None:
        # Print Output in Simulation Environment
        if not RobotBase.isReal():
            #if not self.noPrint: print( states )
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


    def setMotionMagic(self) -> None:
        mmValue = self.motionMagic.get()
        self.moduleFL.setMotionMagic( mmValue )
        self.moduleFR.setMotionMagic( mmValue )
        self.moduleBL.setMotionMagic( mmValue )
        self.moduleBR.setMotionMagic( mmValue )
