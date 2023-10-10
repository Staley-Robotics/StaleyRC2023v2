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
extend_kP = 0.5
extend_kI = 0
extend_kD = 0
extend_kF = 0
extend_kError = 0

# Position Constants
extend_position_min = 0
extend_position_max = 100
extend_position_start = 5

# Motion Magic Constants
extend_mmMaxVelocity = 2048
extend_mmMaxAcceleration = 2048
extend_mmSCurveSmoothing = 0
extend_kMaxSpeedMetersPerSecond = 3
extend_kMaxAccelMetersPerSecondSq = 3
extend_kMaxAngularSpeedMetersPerSecond = math.pi
extend_kMaxAngularAccelMetersPerSecondSq = math.pi


class ArmExtend(SubsystemBase):
    currentSetPosition:int = 0
    # Variables
    extendMotor:WPI_TalonSRX = None
    stallDetector:MotorUtils = MotorUtils(0, 0.15, 0.15, PowerDistribution(0, PowerDistribution.ModuleType.kCTRE) )

    # Constructor
    def __init__(self):
        super().__init__()

        # Extend Motor
        self.extendMotor = WPI_TalonSRX(31)
        self.extendMotor.configFactoryDefault()
        self.extendMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0)
        self.extendMotor.setSensorPhase(False)
        self.extendMotor.setInverted(False)
        self.extendMotor.setNeutralMode(NeutralMode.Brake)
        self.extendMotor.selectProfileSlot(1, 0)
        self.extendMotor.configNeutralDeadband(0.001)
        
        #self.extendMotor.configFeedbackNotContinuous(True)

        # Extend PID
        self.extendMotor.config_kP( 1, extend_kP )
        self.extendMotor.config_kI( 1, extend_kI )
        self.extendMotor.config_kD( 1, extend_kD )
        self.extendMotor.config_kF( 1, extend_kF )
        self.extendMotor.configAllowableClosedloopError( 1, extend_kError )

        #self.extendMotor.configMotionCruiseVelocity( drive_mmMaxVelocity )
        #self.extendMotor.configMotionAcceleration( drive_mmMaxAcceleration )
        #self.extendMotor.configMotionSCurveStrength( drive_mmSCurveSmoothing )


        # Set Starting Position
        self.setPosition( extend_position_start )

        #self.getController().initSendable()
        self.setSubsystem( "ArmExtend" )
        self.setName( "ArmExtend" )
        self.addChild( "Extend", self.extendMotor )

    def periodic(self) -> None:
        kMeasuredPosHorizontal:int = 840; #Position measured when arm is horizontal
        kTicksPerDegree:float = 4096 / 360 #Sensor is 1:1 with arm rotation
        currentPos:int = self.extendMotor.getSelectedSensorPosition()
        degrees:float = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree
        radians:float = math.radians(degrees)
        cosineScalar:float = math.cos(radians)

        self.extendMotor.set(
            ControlMode.MotionMagic,
            self.currentSetPosition,
            DemandType.ArbitraryFeedForward,
            extend_kF * cosineScalar
        )

    ### Extend Position Functions
    # Sets the Extend Position
    def setPosition(self, position:int) -> None:
        position = max( min( position, extend_position_max ), extend_position_min ) # Verify position is in range
        #self.extendMotor.set(ControlMode.Position, position)
        self.currentSetPosition = position
        print( f"Extend: {position}" )

    # Get the Current Extend Position
    def getPosition(self) -> int:
        position = self.extendMotor.getClosedLoopTarget() - self.extendMotor.getClosedLoopError()
        return int( position )
    
    # Get the Extend Position Target 
    def getPositionTarget(self) -> int:
        return int( self.extendMotor.getClosedLoopTarget() )

    # Is the Extend Motor at the set Position?
    def atPosition(self) -> bool:
        atPosition = abs( self.extendMotor.getClosedLoopError() ) < extend_kError
        return atPosition
