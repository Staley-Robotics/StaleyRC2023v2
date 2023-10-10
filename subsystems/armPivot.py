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
from commands2 import SubsystemBase, PIDSubsystem
from ctre import WPI_TalonFX, WPI_TalonSRX, FeedbackDevice, RemoteFeedbackDevice, NeutralMode, ControlMode, DemandType
from wpilib import PowerDistribution, RobotBase, RobotState
#from wpilib.PowerDistribution import ModuleType
from wpimath.controller import PIDController

# Our Imports
from util.MotorUtils import MotorUtils


### Constants
# Module Physical Constants

# Controller Constants
pivot_kP = 0.15
pivot_kI = 0
pivot_kD = 0
pivot_kF = 0.065
pivot_kError = 0.5

# Position Constants
pivot_position_min = 0
pivot_position_max = 100
pivot_position_start = 5

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
    stallDetector:MotorUtils = MotorUtils(0, 0.15, 0.15, PowerDistribution(0, PowerDistribution.ModuleType.kCTRE) )
    currentSetPosition:int = 0

    def __init__(self):
        super().__init__()
        # Pivot Motor
        self.pivotMotor = WPI_TalonFX(9, "rio")
        self.pivotMotor.configFactoryDefault()
        self.pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0)
        self.pivotMotor.setSensorPhase(True)
        self.pivotMotor.setInverted(False)
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
        self.setPosition( pivot_position_start )

        #self.getController().initSendable()
        self.setSubsystem( "ArmPivot" )
        self.setName( "ArmPivot" )
        self.addChild( "PivotMotor", self.pivotMotor )

    noPrint = False
    def periodic(self) -> None:
        if RobotState.isDisabled(): return None

        kMeasuredPosHorizontal:int = 90; #Position measured when arm is horizontal
        kTicksPerDegree:float = 1 #4096 / 360 #Sensor is 1:1 with arm rotation
        currentPos:int = self.pivotMotor.getSelectedSensorPosition()
        degrees:float = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree
        radians:float = math.radians(degrees)
        cosineScalar:float = math.cos(radians)

        self.pivotMotor.set(
            ControlMode.MotionMagic,
            self.currentSetPosition,
            DemandType.ArbitraryFeedForward,
            pivot_kF * cosineScalar
        )

        #if not RobotBase.isReal():
        #    if not self.noPrint: print( f"Pivot: {self.pivotMotor.getSelectedSensorPosition()}\tTarget: {self.pivotMotor.getClosedLoopTarget()}\tError: {self.pivotMotor.getClosedLoopError()}" )
        #    if self.pivotMotor.getClosedLoopError() > 0.0:
        #        self.noPrint = False
        #    else:
        #        self.noPrint = True



    ### Pivot Position Functions
    # Set Pivot Position
    def setPosition(self, position:int) -> None:
        position = max( min( position, pivot_position_max ), pivot_position_min ) # Verify position is in range
        self.currentSetPosition = position
        #self.pivotMotor.set(ControlMode.Position, position)  ### Using periodic property
        #print( f"Pivot: {position}" )

    # Get the Current Pivot Position
    def getPosition(self) -> int:
        position = self.pivotMotor.getClosedLoopTarget() - self.pivotMotor.getClosedLoopError()
        return int( position )
    
    # Get the Pivot Position Target 
    def getPositionTarget(self) -> int:
        return int( self.pivotMotor.getClosedLoopTarget() )
    
    # Is the Pivotr Motor at the set Position?
    def atPosition(self) -> bool:
        messurement = self.pivotMotor.getSelectedSensorPosition(0) - self.pivotMotor.getClosedLoopTarget(0)
        atPosition:bool = abs( messurement ) <= pivot_kError
        return atPosition
