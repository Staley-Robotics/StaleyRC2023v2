"""
Description: Pivot Arm
Version:  1
Date:  2023-10-03

Dependencies:
- None

CAN Network:  Default / RIO
Motors:  TalonFX
Sensors:  TalonSRX connected to Mag Encoder
"""


### Imports
# Python Imports
import typing
import math

# FRC Component Imports
from commands2 import SubsystemBase
from ctre import WPI_TalonFX, WPI_TalonSRX, FeedbackDevice, RemoteFeedbackDevice, NeutralMode, ControlMode, DemandType
from wpilib import RobotState
from ntcore import *

# Our Imports



### Constants
# Module Physical Constants
pivot_ticks:int = 4096
pivot_manualMoveRate:int = 500 # Ticks Per 20 ms (total distance in 0.5 seconds)

# Controller Constants
pivot_kP = 1.75 # 1.5
pivot_kI = 0.0005
pivot_kD = 9.0
pivot_kF = 0.01
pivot_kError = 20

# Position Constants
pivot_position_min = 2325 # 10.0 degrees
pivot_position_horizontal = 2438 # 0.0 degrees
pivot_position_max = 3440 # -88.0 degrees

# Motion Magic Constants
pivot_mmMaxVelocity = 8192 #2048
pivot_mmMaxAcceleration = 2 * pivot_mmMaxVelocity # Reach Max Speed in 1/2 second
pivot_mmSCurveSmoothing = 8 # 0-8 Motion Magic Smoothness

# ArmPivot Subsystem Class
class ArmPivot(SubsystemBase):
    # Variables
    __motor__:WPI_TalonFX = None
    __position__:int = 0
    __ntTbl__:NetworkTable = NetworkTableInstance.getDefault().getTable( "ArmPivot" )

    # Initialization
    def __init__(self):
        # Pivot Sensor
        self.pivotSensor:WPI_TalonSRX = WPI_TalonSRX( 10 ) #, "rio" )
        self.pivotSensor.configFactoryDefault()
        self.pivotSensor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0)
        self.pivotSensor.setSensorPhase(True)
        self.pivotSensor.setInverted(True)

        # Pivot Motor
        __motor__ = WPI_TalonFX(9, "rio")
        __motor__.configFactoryDefault()
        __motor__.configRemoteFeedbackFilter( self.pivotSensor, 0 )
        __motor__.configSelectedFeedbackSensor( RemoteFeedbackDevice.RemoteSensor0, 0 )
        __motor__.setSensorPhase(True)
        __motor__.setInverted(True)
        __motor__.setNeutralMode(NeutralMode.Coast)
        __motor__.selectProfileSlot(1, 0)
        __motor__.configNeutralDeadband(0.001)

        # Pivot PID
        __motor__.config_kP( 1, pivot_kP )
        __motor__.config_kI( 1, pivot_kI )
        __motor__.config_kD( 1, pivot_kD )
        __motor__.config_kF( 1, pivot_kF )
        __motor__.configAllowableClosedloopError( 1, pivot_kError )

        # Motion Magic
        __motor__.configMotionCruiseVelocity( pivot_mmMaxVelocity )
        __motor__.configMotionAcceleration( pivot_mmMaxAcceleration )
        __motor__.configMotionSCurveStrength( pivot_mmSCurveSmoothing )

        # Set Starting Position
        self.setPosition( __motor__.getSelectedSensorPosition(0) )

        # Subsystem Setup
        super().__init__()
        self.setSubsystem( "ArmPivot" )
        self.setName( "ArmPivot" )
        self.addChild( "PivotMotor", __motor__ )

        # Save to Global Variables
        self.__motor__ = __motor__

    # Periodic Loop (Runs every iteration of the )
    def periodic(self) -> None:
        # Put Telemetry on Shuffleboard
        self.__ntTbl__.putNumber( "Position", self.getPosition() )
        self.__ntTbl__.putNumber( "PositionDegrees", self.getPositionDegrees() )
        self.__ntTbl__.putNumber( "CodeTarget", self.__position__ )
        self.__ntTbl__.putNumber( "Target", self.getTargetPosition() )
        self.__ntTbl__.putNumber( "Error", self.__motor__.getClosedLoopError(0) )


    def update(self):      
        # Don't Run Rest of Periodic While Disabled
        #if RobotState.isDisabled(): return None

        # Gravity Compensation
        pos:float = self.getPositionRadians()
        cosineScalar:float = math.cos(pos)
        
        # Set Motor
        self.__motor__.set(
            #ControlMode.Position, 
            ControlMode.MotionMagic,
            self.__position__,
            DemandType.ArbitraryFeedForward,
            pivot_kF * cosineScalar
        )

    ### Pivot Position Functions
    # Set Pivot Position in Sensor Units
    def setPosition(self, ticks:int) -> None:
        self.__position__ = min( max( ticks, pivot_position_min ), pivot_position_max )

    # Get the Current Pivot Position in Sensor Units
    def getPosition(self) -> int:
        position = self.__motor__.getSelectedSensorPosition(0)
        return int( position )

    # Move the Current Pivot Position
    def movePosition(self, input:float) -> None:
        position = self.getPosition() - int( input * pivot_manualMoveRate )
        self.setPosition( position )

    # Set Pivot Position in Degrees
    def setPositionDegrees(self, degrees:float) -> None:
        ticks:int = pivot_position_horizontal - ( degrees / 360.0 * pivot_ticks )
        self.setPosition( ticks )

    # Get the Current Pivot Position in Degrees
    def getPositionDegrees(self) -> float:
        ticks = self.getPosition()
        degrees:float = ( pivot_position_horizontal - ticks ) / pivot_ticks * 360.0
        degrees = round( degrees, 3 )
        return float( degrees )
    
    # Get the Current Pivot Position in Radians
    def getPositionRadians(self) -> float:
        degrees = self.getPositionDegrees()
        radians = math.radians( degrees )
        return float( radians )

    # Get the Pivot Position Target 
    def getTargetPosition(self) -> int:
        return int( self.__motor__.getClosedLoopTarget() )
    
    # Get the Pivot Position Target in Degrees
    def getTargetPositionDegrees(self) -> float:
        ticks = self.getTargetPosition()
        degrees:float = ( pivot_position_horizontal - ticks ) / pivot_ticks * 360.0
        degrees = round( degrees, 3 )
        return float( degrees )
    
    # Is the Pivotr Motor at the set Position?
    def atPosition(self) -> bool: 
        atPosition = abs( self.__position__ - self.getPosition() ) < pivot_kError
        return atPosition
