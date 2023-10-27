"""
Description: Swerve Module (West Coast Products - Swerve X)
Version:  1
Date:  2023-09-29

Drive Motor Controllers:  TALON/FALCON FX
Angle Motor Controllers:  TALON/FALCON FX
Angle Sensors:  CAN CODERS

Velocity: Closed Loop (FALCON Integrated)
Angle:  Closed Loop (FALCON Integrated)
"""

### Imports
# Python Imports
import math

# FRC Component Imports
from commands2 import SubsystemBase
from ctre import WPI_TalonFX, ControlMode, FeedbackDevice, RemoteFeedbackDevice, NeutralMode, TalonFXPIDSetConfiguration
from ctre.sensors import WPI_CANCoder, SensorInitializationStrategy, AbsoluteSensorRange
from wpilib import RobotBase, RobotState
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d
from wpiutil import *

### Constants
# Module Physical Constants
driveMotorTicks = 2048
angleMotorTicks = 2048
angleSensorTicks = 4096
driveGearRatio = 1/7.36 #Flipped Pulley (5.50, 6.55, 7.80), Flipped Gear (6.75, 7.36, 8.10)
wheelRadius = 0.0508

# Controller Constants
drive_kP = 0.15
drive_kI = 0
drive_kD = 0
drive_kF = 0.065
drive_kSlotIdx = 0
drive_mmMaxVelocity = 20480
drive_mmMaxAcceleration = 4 * drive_mmMaxVelocity
drive_mmSCurveSmoothing = 8

angle_kP = 0.5
angle_kI = 0
angle_kD = 0
angle_kF = 0
angle_kSlotIdx = 0
angle_mmMaxVelocity = 2048
angle_mmMaxAcceleration = 2 * angle_mmMaxVelocity
angle_mmSCurveSmoothing = 8

# Class: SwerveModule
class SwerveModule(SubsystemBase):
    motionMagic:bool = False

    driveMotor:WPI_TalonFX = None
    angleMotor:WPI_TalonFX = None
    angleSensor:WPI_CANCoder = None

    modulePosition:Translation2d = None
    moduleState:SwerveModuleState = None

    def __init__(self, subsystemName, driveId, angleId, sensorId, posX, posY, angleOffset):
        super().__init__()

        # Angle Sensor
        self.angleSensor = WPI_CANCoder( sensorId, "canivore1")
        self.angleSensor.configFactoryDefault()
        self.angleSensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        self.angleSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.angleSensor.configSensorDirection(False)
        if not RobotBase.isSimulation(): self.angleSensor.setPosition(self.angleSensor.getAbsolutePosition() - angleOffset)

        # Angle Motor
        self.angleMotor = WPI_TalonFX( angleId, "canivore1")
        self.angleMotor.configFactoryDefault()
        self.angleMotor.configRemoteFeedbackFilter(self.angleSensor, 0)
        self.angleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0)
        self.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.None_, 1)
        self.angleMotor.setInverted(True)
        self.angleMotor.setSensorPhase(True)
        self.angleMotor.setNeutralMode(NeutralMode.Coast)
        self.angleMotor.configFeedbackNotContinuous(True)
        self.angleMotor.configNeutralDeadband(0.001)

        # Drive Motor
        self.driveMotor = WPI_TalonFX( driveId, "canivore1")
        self.driveMotor.configFactoryDefault()
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        self.driveMotor.setInverted(True)
        self.driveMotor.setSensorPhase(False)
        self.driveMotor.setNeutralMode(NeutralMode.Coast)
        self.driveMotor.configNeutralDeadband(0.001)

        # Subsystem Dashboards
        self.setSubsystem( f"SwerveModule/{subsystemName}" )
        self.setName( f"SwerveModule/{subsystemName}" )
        self.addChild( f"Drive", self.driveMotor )
        self.addChild( f"Angle", self.angleMotor )
        self.addChild( f"Sensor", self.angleSensor )

        # Angle Integrated PID Controller
        self.angleMotor.config_kP( angle_kSlotIdx, angle_kP )
        self.angleMotor.config_kI( angle_kSlotIdx, angle_kI )
        self.angleMotor.config_kD( angle_kSlotIdx, angle_kD )
        self.angleMotor.config_kF( angle_kSlotIdx, angle_kF )
        self.angleMotor.selectProfileSlot(angle_kSlotIdx, 0)

        # Drive Integrated PID Controller
        self.driveMotor.config_kP( drive_kSlotIdx, drive_kP )
        self.driveMotor.config_kI( drive_kSlotIdx, drive_kI )
        self.driveMotor.config_kD( drive_kSlotIdx, drive_kP )
        self.driveMotor.config_kF( drive_kSlotIdx, drive_kF )
        self.driveMotor.selectProfileSlot(drive_kSlotIdx, 0)

        # Drive Integrated PID - Motion Magic Properties
        self.angleMotor.configMotionCruiseVelocity( angle_mmMaxVelocity )
        self.angleMotor.configMotionAcceleration( angle_mmMaxAcceleration )
        self.angleMotor.configMotionSCurveStrength( angle_mmSCurveSmoothing )

        # Drive Integrated PID - Motion Magic Properties
        self.driveMotor.configMotionCruiseVelocity( drive_mmMaxVelocity )
        self.driveMotor.configMotionAcceleration( drive_mmMaxAcceleration )
        self.driveMotor.configMotionSCurveStrength( drive_mmSCurveSmoothing )

        # Module Position
        self.modulePosition = Translation2d( posX, posY )
        
        # Module State
        self.moduleState = SwerveModuleState( 0, Rotation2d(0) )

    def periodic(self) -> None:
        pass

    def setDesiredState(self, desiredState:SwerveModuleState):
        ### Calculate / Optomize
        currentAnglePosition = self.angleSensor.getPosition()
        currentAngleRotation = Rotation2d(0).fromDegrees(currentAnglePosition)
        optimalState:SwerveModuleState = SwerveModuleState.optimize(
            desiredState,
            currentAngleRotation
        )
        self.moduleState = optimalState

        # Velocity Calculate
        velocity = optimalState.speed
        velocityTp100ms = getVelocityMpsToTp100ms(velocity, driveMotorTicks, wheelRadius, driveGearRatio ) # Get Velocity in Ticks per 100 ms
        # Set Velocity
        if self.motionMagic:
            # Convert Velocity to Predicted Position
            measuredPosition = self.driveMotor.getSelectedSensorPosition(0)
            targetPosition = measuredPosition + ( velocityTp100ms / 5 )
            self.driveMotor.set( ControlMode.MotionMagic, targetPosition )
        else:
            self.driveMotor.set( ControlMode.Velocity, velocityTp100ms )

        # Angle Position Calculate
        currentTicks = self.angleMotor.getSelectedSensorPosition(0)
        targetTicks = getTicksFromRotation(optimalState.angle, angleSensorTicks)  # Get Optomized Target as Ticks
        positionTicks = getContinuousInputMeasurement(currentTicks, targetTicks, angleSensorTicks)  # Correction for [-180,180)
        # Set Angle Position
        if self.motionMagic:
            self.angleMotor.set( ControlMode.MotionMagic, positionTicks )
        else:
            self.angleMotor.set( ControlMode.Position, positionTicks )

    def getModulePosition(self) -> Translation2d:
        return self.modulePosition
    
    def getModuleState(self) -> SwerveModuleState:
        return self.moduleState

    def getPosition(self) -> SwerveModulePosition:
        dPosition = self.driveMotor.getSelectedSensorPosition(0)
        driveMeters = getDistanceTicksToMeters(dPosition, driveMotorTicks, wheelRadius, driveGearRatio)
        rPosition = self.angleMotor.getSelectedSensorPosition(0)  ## Should we use self.angleSensor ??
        rotatePosition = getRotationFromTicks(rPosition, angleSensorTicks)
        #rPosition = self.angleSensor.getPosition()  ## Should we use self.angleSensor ??
        #rotatePosition = Rotation2d(0).fromDegrees(rPosition)

        return SwerveModulePosition(
            distance=driveMeters,
            angle=rotatePosition
        )
    
    def setMotionMagic(self, state:bool):
        self.motionMagic = state



### Helper Functions
# Ticks From Rotation Converter Calculator
def getTicksFromRotation( rotation:Rotation2d, ticksPerRotation:float ) -> int:
    radians:float = rotation.radians()
    ticks:int = int( radians * ticksPerRotation / ( 2 * math.pi ) )
    return ticks

# Rotations From Ticks Converter Calculator
def getRotationFromTicks( ticks:float, ticksPerRotation:float ) -> Rotation2d:
    radians:float = ticks * ( 2 * math.pi ) / ticksPerRotation
    rotation = Rotation2d( value=radians )
    return rotation

# Velocity Ticks Per 100 ms to Meters Per Second Calculator
def getVelocityTp100msToMps( ticksPer100ms:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    ticksPerSec:float = ticksPer100ms * 10
    rotationsPerSec:float = ticksPerSec / ticksPerRotation
    metersPerSec:float = rotationsPerSec * ( 2 * math.pi * radius * gearRatio)
    return metersPerSec

# Velcotiy Meters Per Second to Ticks Per 100 ms Calculator
def getVelocityMpsToTp100ms( metersPerSec:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotationsPerSec:float = metersPerSec / ( 2 * math.pi * radius * gearRatio )
    ticksPerSec:float = rotationsPerSec * ticksPerRotation
    ticksPer100ms:float = ticksPerSec / 10
    return ticksPer100ms

# Ticks to Meters Calculator
def getDistanceTicksToMeters( ticks:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotations:float = ticks / ticksPerRotation
    meters:float = rotations * ( 2 * math.pi * radius * gearRatio )
    return meters

# Meters to Ticks Calculator
def getDistanceMetersToTicks( meters:float, ticksPerRotation:float, radius:float, gearRatio: float = 1.0 ) -> float:
    rotations:float = meters / ( 2 * math.pi * radius * gearRatio )
    ticks:float = rotations * ticksPerRotation
    return ticks

# Continous Input Measurement Calculator
def getContinuousInputMeasurement( currentSetpoint:float, targetSetpoint:float, measurement:float ) -> float:
    targetSetpoint = math.remainder(targetSetpoint, measurement)
    remainder:float = currentSetpoint % measurement
    adjustedAngleSetpoint:float = targetSetpoint + (currentSetpoint - remainder)

    if (adjustedAngleSetpoint - currentSetpoint > (measurement/2)):
        adjustedAngleSetpoint -= measurement
    elif (adjustedAngleSetpoint - currentSetpoint < (-measurement/2)):
        adjustedAngleSetpoint += measurement

    return adjustedAngleSetpoint
