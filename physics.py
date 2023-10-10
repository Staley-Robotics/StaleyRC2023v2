from pyfrc.physics.core import PhysicsInterface
from ctre import *
from ctre.sensors import *
import math
from wpilib import RobotState

class PhysicsEngine:
    def __init__(self,physics_controller,robot):
        self.physics_controller:PhysicsInterface = physics_controller

        self.pivotMotor:WPI_TalonFX = robot.m_robotContainer.armPivot.pivotMotor
        self.pivotMotorSim:TalonFXSimCollection = robot.m_robotContainer.armPivot.pivotMotor.getSimCollection()
        self.pivotMotorSim.setIntegratedSensorRawPosition(5)

        self.extendMotor:WPI_TalonSRX = robot.m_robotContainer.armExtend.extendMotor
        self.extendMotorSim:TalonSRXSimCollection = robot.m_robotContainer.armExtend.extendMotor.getSimCollection()
        self.extendMotorSim.setQuadratureRawPosition(5)

    def update_sim(self, now, tm_diff):
        if RobotState.isDisabled(): return None

        pivotTarget:int = int(self.pivotMotor.getClosedLoopTarget())
        #self.pivotMotorSim.setIntegratedSensorRawPosition( pivotTarget )
        pivotError:int = int(self.pivotMotor.getClosedLoopError())
        add = max( min( pivotError, 1 ), -1 )
        self.pivotMotorSim.addIntegratedSensorPosition( add )

        extendTarget:int = int(self.extendMotor.getClosedLoopTarget()  )
        self.extendMotorSim.setQuadratureRawPosition( extendTarget )
