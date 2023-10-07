from pyfrc.physics.core import PhysicsInterface
from ctre import *
from ctre.sensors import *

class PhysicsEngine:
    def __init__(self,physics_controller,robot):
        self.physics_controller:PhysicsInterface = physics_controller

        self.pivotMotor:WPI_TalonFX = robot.m_robotContainer.arm.pivotMotor
        self.pivotMotorSim:TalonFXSimCollection = robot.m_robotContainer.arm.pivotMotor.getSimCollection()
        self.pivotMotorSim.setIntegratedSensorRawPosition(5)

        self.extendMotor:WPI_TalonSRX = robot.m_robotContainer.arm.extendMotor
        self.extendMotorSim:TalonSRXSimCollection = robot.m_robotContainer.arm.extendMotor.getSimCollection()
        self.extendMotorSim.setQuadratureRawPosition(5)

    def update_sim(self, now, tm_diff):
        pivotTarget:int = int(self.pivotMotor.getClosedLoopTarget()  )
        self.pivotMotorSim.setIntegratedSensorRawPosition( pivotTarget )

        extendTarget:int = int(self.extendMotor.getClosedLoopTarget()  )
        self.extendMotorSim.setQuadratureRawPosition( extendTarget )
