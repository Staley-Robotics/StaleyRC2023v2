"""
Description: Pivot + Extending Arm
Version:  1
Date:  2023-10-03

Dependencies:
- None

CAN Network:  Default / RIO
"""

### Imports
# Python Imports
import math

# FRC Component Imports
from commands2 import SubsystemBase
from ctre import WPI_TalonSRX, FeedbackDevice, NeutralMode, ControlMode
from wpilib import RobotState
from ntcore import *

# Our Imports


### Constants
# Module Physical Constants
extend_manualMoveRate:int = 4500 # Ticks Per 20ms (total distance in 0.5 seconds)

# Controller Constants
extend_kP = 0.30
extend_kI = 0
extend_kD = 0
extend_kF = 0 #0.065
extend_kError = 10

# Position Constants
extend_position_min = 0
extend_position_max = 24400 #22000
extend_position_start = 5

extend_length_inches = 16.6 #15.0
extend_ticksPerInch = ( extend_position_max - extend_position_min ) / extend_length_inches

# Motion Magic Constants
extend_mmMaxVelocity = 2048
extend_mmMaxAcceleration = 2 * extend_mmMaxVelocity # Reach Max Speed in 1/2 second
extend_mmSCurveSmoothing = 8 # 0-8 Motion Magic Smoothness

# ArmExtend Subsystem Class
class ArmExtend(SubsystemBase):
    # Variables
    __motor__:WPI_TalonSRX = None
    __position__:int = 0
    __ntTbl__:NetworkTable = NetworkTableInstance.getDefault().getTable( "ArmExtend" )

    # Initialization
    def __init__(self):
        # Extend Motor
        __motor__:WPI_TalonSRX = WPI_TalonSRX(31)
        __motor__.configFactoryDefault()
        __motor__.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0)
        __motor__.setSensorPhase(False)
        __motor__.setInverted(False)
        __motor__.setNeutralMode(NeutralMode.Brake)
        __motor__.selectProfileSlot(1, 0)
        __motor__.configNeutralDeadband(0.001)
        __motor__.setSelectedSensorPosition(0)

        # Extend PID
        __motor__.config_kP( 1, extend_kP )
        __motor__.config_kI( 1, extend_kI )
        __motor__.config_kD( 1, extend_kD )
        __motor__.config_kF( 1, extend_kF )
        __motor__.configAllowableClosedloopError( 1, extend_kError )

        # Motion Magic
        __motor__.configMotionCruiseVelocity( extend_mmMaxVelocity )
        __motor__.configMotionAcceleration( extend_mmMaxAcceleration )
        __motor__.configMotionSCurveStrength( extend_mmSCurveSmoothing )

        # Set Starting Position
        self.setPosition( __motor__.getSelectedSensorPosition(0) )

        # Subsystem Setup
        super().__init__()
        self.setSubsystem( "ArmExtend" )
        self.setName( "ArmExtend" )
        self.addChild( "ExtendMotor", __motor__ )

        # Save to Global Variables
        self.__motor__ = __motor__

    def periodic(self) -> None:
        # Put Telemetry on Shuffleboard
        self.__ntTbl__.putNumber( "Position", self.getPosition() )
        self.__ntTbl__.putNumber( "PositionInches", self.getPositionInches() )
        self.__ntTbl__.putNumber( "CodeTarget", self.__position__ )
        self.__ntTbl__.putNumber( "Target", self.getTargetPosition() )
        self.__ntTbl__.putNumber( "Error", self.__motor__.getClosedLoopError(0) )

    def update(self) -> None:
        # Set Motor
        self.__motor__.set(
            ControlMode.Position,
            self.__position__
        )

    ### Extend Position Functions
    # Sets the Extend Position in Sensor Units
    def setPosition(self, position:int, override:bool = False) -> None:
        if not override:
            position = min( max( position, extend_position_min ), extend_position_max )
        self.__position__ = position

    # Get the Current Extend Position in Sensor Units
    def getPosition(self) -> int:
        position = self.__motor__.getSelectedSensorPosition(0)
        return int( position )

    # Move Position by Joystick Float
    def movePosition(self, input:float) -> None:
        position = self.getPosition() + int( input * extend_manualMoveRate )
        self.setPosition( position )

    # Reset the Current Position and Sensor Settings
    def resetPosition(self) -> None:
        self.__motor__.setSelectedSensorPosition( 0 )
        self.__position__ = 0

    # Set Extend Position in Inches
    def setPositionInches(self, inches:float) -> None:
        ticks:int = inches * extend_ticksPerInch
        self.setPosition( ticks )

    # Get Extend Position in Inches
    def getPositionInches(self) -> float:
        ticks = self.getPosition()
        inches:float = float( ticks / extend_ticksPerInch )
        inches = round( inches, 3 )
        return float( inches )

    # Get Extend Position Target
    def getTargetPosition(self) -> int:
        return int( self.__motor__.getClosedLoopTarget() )
    
    # Get Extend Position Target in Inches
    def getTargetPositionInches(self) -> float:
        ticks = self.getTargetPosition()
        inches:float = float( ticks / extend_ticksPerInch )
        inches = round( inches, 3 )
        return float( inches )
    
    # Is the Extend Motor at the set Position?
    def atPosition(self) -> bool:
        atPosition = abs( self.__position__ - self.getPosition() ) < extend_kError
        return atPosition
