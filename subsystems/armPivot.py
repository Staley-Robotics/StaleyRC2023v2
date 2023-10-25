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
import typing
import math

# FRC Component Imports
from commands2 import SubsystemBase
from ctre import WPI_TalonFX, TalonSRX, WPI_TalonSRX, FeedbackDevice, RemoteFeedbackDevice, NeutralMode, ControlMode, DemandType
from wpilib import RobotBase, RobotState, Mechanism2d, MechanismLigament2d, MechanismObject2d, MechanismRoot2d, SmartDashboard

# Our Imports



### Constants
# Module Physical Constants

# Controller Constants
pivot_kP = 1.50
pivot_kI = 0
pivot_kD = 15.00
pivot_kF = 0 # 0.065
pivot_kError = 10

# Position Constants
pivot_position_min = -87.5
pivot_position_max = 7.0
pivot_position_start = 0.0

pivot_ticks = 4096
pivot_offset = 2438

# Motion Magic Constants
pivot_mmMaxVelocity = 2048
pivot_mmMaxAcceleration = 2048
pivot_mmSCurveSmoothing = 0
pivot_kMaxSpeedMetersPerSecond = 3
pivot_kMaxAccelMetersPerSecondSq = 3
pivot_kMaxAngularSpeedMetersPerSecond = math.pi
pivot_kMaxAngularAccelMetersPerSecondSq = math.pi

class ArmPivot(SubsystemBase):
    # Variables
    pivotMotor:WPI_TalonFX = None
    #stallDetector:MotorUtils = MotorUtils(0, 0.15, 0.15, PowerDistribution(0, PowerDistribution.ModuleType.kCTRE) )
    currentSetPosition:int = 0

    def __init__(self):
        super().__init__()
        # Pivot Sensor
        self.pivotSensor:WPI_TalonSRX = WPI_TalonSRX( 10 ) #, "rio" )
        self.pivotSensor.configFactoryDefault()
        self.pivotSensor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0)
        self.pivotSensor.setSensorPhase(True)
        self.pivotSensor.setInverted(True)

        # Pivot Motor
        self.pivotMotor = WPI_TalonFX(9, "rio")
        self.pivotMotor.configFactoryDefault()
        #self.pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0) # For Integrated Sensor
        self.pivotMotor.configRemoteFeedbackFilter( self.pivotSensor, 0 ) # For Mag Encoder
        self.pivotMotor.configSelectedFeedbackSensor( RemoteFeedbackDevice.RemoteSensor0, 0 ) # For Mag Encoder
        self.pivotMotor.setSensorPhase(True)
        self.pivotMotor.setInverted(True)
        self.pivotMotor.setNeutralMode(NeutralMode.Brake)
        self.pivotMotor.selectProfileSlot(1, 0)
        self.pivotMotor.configNeutralDeadband(0.001)

        # Pivot PID
        self.pivotMotor.config_kP( 1, pivot_kP )
        self.pivotMotor.config_kI( 1, pivot_kI )
        self.pivotMotor.config_kD( 1, pivot_kD )
        self.pivotMotor.config_kF( 1, pivot_kF )
        self.pivotMotor.configAllowableClosedloopError( 1, pivot_kError )

        # Motion Magic
        self.pivotMotor.configMotionCruiseVelocity( pivot_mmMaxVelocity )
        self.pivotMotor.configMotionAcceleration( pivot_mmMaxAcceleration )
        self.pivotMotor.configMotionSCurveStrength( pivot_mmSCurveSmoothing )

        # Set Starting Position
        #self.currentSetPosition = pivot_position_start
        self.currentSetPosition = -20 #-(self.pivotMotor.getSelectedSensorPosition(0) - pivot_offset) / pivot_ticks * 360
        #self.setPosition( pivot_position_start )

        #self.getController().initSendable()
        self.setSubsystem( "ArmPivot" )
        self.setName( "ArmPivot" )
        self.addChild( "PivotMotor", self.pivotMotor )

    noPrint = False
    def periodic(self) -> None:
        SmartDashboard.putNumber( "ArmPivot/Position", self.pivotMotor.getSelectedSensorPosition(0) )
        SmartDashboard.putNumber( "ArmPivot/SensorPosition", self.pivotSensor.getSelectedSensorPosition(0) )
        SmartDashboard.putNumber( "ArmPivot/Target", self.pivotMotor.getClosedLoopTarget(0) )
        SmartDashboard.putNumber( "ArmPivot/Error", self.pivotMotor.getClosedLoopError(0) )
        SmartDashboard.putNumber( "ArmPivot/InternalPos", self.currentSetPosition)
        if RobotState.isDisabled(): return None

        kMeasuredPosHorizontal:int = 100000; #Position measured when arm is horizontal
        kTicksPerDegree:float = 1024 #4096 / 360 #Sensor is 1:1 with arm rotation
        currentPos:int = self.pivotMotor.getSelectedSensorPosition()
        degrees:float = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree
        radians:float = math.radians(degrees)
        cosineScalar:float = math.cos(radians)

        self.currentSetPosition = min( max( self.currentSetPosition, pivot_position_min ), pivot_position_max )
        pos = self.currentSetPosition / -360 * pivot_ticks + pivot_offset
        #print( self.currentSetPosition )
        self.pivotMotor.set(
            #ControlMode.MotionMagic,
            ControlMode.Position,
            pos #,
            #DemandType.ArbitraryFeedForward,
            #pivot_kF * cosineScalar
        )

        if not RobotBase.isReal():
            #if not self.noPrint: print( f"Pivot: {self.pivotMotor.getSelectedSensorPosition()}\tTarget: {self.pivotMotor.getClosedLoopTarget()}\tError: {self.pivotMotor.getClosedLoopError()}" )
            if self.pivotMotor.getClosedLoopError() > 0.0:
                self.noPrint = False
            else:
                self.noPrint = True



    ### Pivot Position Functions
    # Set Pivot Position
    def setPosition(self, position:float) -> None:
        position = max( min( position, pivot_position_max ), pivot_position_min ) # Verify position is in range
        #scaledPos = (position-2) * 1024
        self.currentSetPosition = position
        #self.pivotMotor.set(ControlMode.Position, position)  ### Using periodic property
        #print( f"Pivot: {position}" )

    def movePosition(self, degreesPerTick:float):
        self.currentSetPosition += degreesPerTick * 4

    # Get the Current Pivot Position
    def getPosition(self) -> int:
        position = self.pivotMotor.getClosedLoopTarget() - self.pivotMotor.getClosedLoopError()
        return int( position )
    
    # Get the Pivot Position Target 
    def getPositionTarget(self) -> int:
        return int( self.pivotMotor.getClosedLoopTarget() )
    
    # Is the Pivotr Motor at the set Position?
    def atPosition(self) -> bool: 
        atPosition = abs( self.pivotMotor.getClosedLoopError() ) < pivot_kError
        return atPosition
