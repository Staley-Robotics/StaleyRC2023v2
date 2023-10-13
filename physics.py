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
        self.extendVeloc:float = 0.0

    def update_sim(self, now, tm_diff):
        if RobotState.isDisabled(): return None

        pivotTarget:int = int(self.pivotMotor.getClosedLoopTarget())
        #self.pivotMotorSim.setIntegratedSensorRawPosition( pivotTarget )
        pivotError:int = int(self.pivotMotor.getClosedLoopError())
        add = max( min( pivotError, 1 ), -1 )
        self.pivotMotorSim.addIntegratedSensorPosition( add )

        #extendTarget:int = int(self.extendMotor.getClosedLoopTarget())
        #self.extendMotorSim.setQuadratureRawPosition( extendTarget )
        #self.extendMotorSim.addQuadraturePosition( extendTarget )
        maxVeloc = 4150
        accelTime = 1.0
        outPerc = self.extendMotorSim.getMotorOutputLeadVoltage() / 12 
        accel = maxVeloc / accelTime * tm_diff * 10 #/ 1000
        theoryVeloc = outPerc * maxVeloc

        if theoryVeloc > self.extendVeloc + accel:
            self.extendVeloc += accel
        elif theoryVeloc < self.extendVeloc - accel:
            self.extendVeloc -= accel
        else:
            self.extendVeloc += 0.9 * ( theoryVeloc - self.extendVeloc )

        current = abs(outPerc) * 30
        statorCurrent = 0 if outPerc ==0 else current / abs(outPerc)

        #print( f"{outPerc}, {accel}, {self.extendVeloc}" )
        self.extendMotorSim.addQuadraturePosition( int(self.extendVeloc * tm_diff * 100) )
        self.extendMotorSim.setQuadratureVelocity( int(self.extendVeloc) )
        self.extendMotorSim.setSupplyCurrent( current )
        self.extendMotorSim.setStatorCurrent( statorCurrent )
        self.extendMotorSim.setBusVoltage( 12 - outPerc * outPerc * 3/4 )
        

