#######################################################
# Description: Pivot + Extending Arm
# Version:  1
# Date:  2023-10-03
#
# Dependencies:
# - None
#
# CAN Network:  Default / RIO
#######################################################

### Imports
# Python Imports
import typing

# FRC Component Imports
from commands2 import SubsystemBase, PIDSubsystem
from ctre import WPI_TalonFX, WPI_TalonSRX, FeedbackDevice, RemoteFeedbackDevice, NeutralMode, ControlMode
from wpilib import PowerDistribution
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
pivot_kError = 0
extend_kP = 0.5
extend_kI = 0
extend_kD = 0
extend_kF = 0
extend_kError = 0

# Position Constants
pivot_position_min = 0
pivot_position_max = 100
pivot_position_start = 5
extend_position_min = 0
extend_position_max = 100
extend_position_start = 5

### Class: Arm
class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()
    def getPosition(self) -> int: pass
    def setPosition(self, position:int): pass

class ArmPivot(Arm):
    # Variables
    pivotMotor:WPI_TalonFX = None
    stallDetector:MotorUtils = MotorUtils(0, 0.15, 0.15, PowerDistribution(0, PowerDistribution.ModuleType.kCTRE) )

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

        # Pivot PID
        self.pivotMotor.config_kP( 1, pivot_kP )
        self.pivotMotor.config_kI( 1, pivot_kI )
        self.pivotMotor.config_kD( 1, pivot_kD )
        self.pivotMotor.config_kF( 1, pivot_kF )
        self.pivotMotor.configAllowableClosedloopError( 0, pivot_kError )

        # Set Starting Position
        self.setPosition( pivot_position_start )

        #self.getController().initSendable()
        self.setSubsystem( "ArmPivot" )
        self.setName( "ArmPivot" )
        self.addChild( "PivotMotor", self.pivotMotor )

    ### Pivot Position Functions
    # Set Pivot Position
    def setPosition(self, position:int) -> None:
        position = max( min( position, pivot_position_max ), pivot_position_min ) # Verify position is in range
        self.pivotMotor.set(ControlMode.Position, position)
        print( f"Pivot: {position}" )

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

class ArmExtend(Arm):
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
        #self.extendMotor.configFeedbackNotContinuous(True)

        # Extend PID
        self.extendMotor.config_kP( 1, extend_kP )
        self.extendMotor.config_kI( 1, extend_kI )
        self.extendMotor.config_kD( 1, extend_kD )
        self.extendMotor.config_kF( 1, extend_kF )
        self.extendMotor.configAllowableClosedloopError( 0, extend_kError )

        # Set Starting Position
        self.setPosition( extend_position_start )

        #self.getController().initSendable()
        self.setSubsystem( "ArmExtend" )
        self.setName( "ArmExtend" )
        self.addChild( "Extend", self.extendMotor )
    
    ### Extend Position Functions
    # Sets the Extend Position
    def setPosition(self, position:int) -> None:
        position = max( min( position, extend_position_max ), extend_position_min ) # Verify position is in range
        self.extendMotor.set(ControlMode.Position, position)
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
