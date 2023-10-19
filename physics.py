from pyfrc.physics.core import PhysicsInterface
from ctre import TalonFX, TalonSRX, TalonFXSimCollection, TalonSRXSimCollection
from ctre.sensors import CANCoder, CANCoderSimCollection
import math
from wpilib import RobotState
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
import typing
from robot import Robot

class SimProfile:
    _lastTime:float
    _running:bool = False

    def getPeriod(self):
        return 0.02

    def run(self):
        pass

class TalonFxSimProfile(SimProfile):
    _motor:TalonFX
    _sim:TalonFXSimCollection
    _accelTime:float
    _fullVel:float
    _sensorPhase:bool 

    _pos:float = 0
    _vel:float = 0.0

    def __init__(self, motor:TalonFX, accelTime:float, maxVelocity:float, sensorPhase:bool):
        self._motor = motor
        self._sim = motor.getSimCollection()
        self._accelTime = accelTime
        self._fullVel = maxVelocity
        self._sensorPhase = sensorPhase

    def run(self, period:float):
        outPerc = self._sim.getMotorOutputLeadVoltage() / 12 
        accel = self._fullVel / self._accelTime * period * 10 #/ 1000
        theoryVeloc = outPerc * self._fullVel

        if theoryVeloc > self._vel + accel:
            self._vel += accel
        elif theoryVeloc < self._vel - accel:
            self._vel -= accel
        else:
            self._vel += 0.9 * ( theoryVeloc - self._vel )

        current = abs(outPerc) * 30
        statorCurrent = 0 if outPerc == 0 else current / abs(outPerc)

        #print( f"{outPerc}, {accel}, {self.extendVeloc}" )
        self._sim.addIntegratedSensorPosition( int(self._vel * period * 100) )
        self._sim.setIntegratedSensorVelocity( int(self._vel) )
        self._sim.setSupplyCurrent( current )
        self._sim.setStatorCurrent( statorCurrent )
        self._sim.setBusVoltage( 12 - outPerc * outPerc * 3/4 )
        pass

class TalonSrxSimProfile(SimProfile):
    _motor:TalonSRX
    _sim:TalonSRXSimCollection
    _accelTime:float
    _fullVel:float
    _sensorPhase:bool 

    _pos:float = 0
    _vel:float = 0.0

    def __init__(self, motor:TalonSRX, accelTime:float, maxVelocity:float, sensorPhase:bool):
        self._motor = motor
        self._sim = motor.getSimCollection()
        self._accelTime = accelTime
        self._fullVel = maxVelocity
        self._sensorPhase = sensorPhase

    def run(self, period:float):
        outPerc = self._sim.getMotorOutputLeadVoltage() / 12 
        if self._sensorPhase: outPerc *= -1

        accel = self._fullVel / self._accelTime * period * 10 #/ 1000
        theoryVeloc = outPerc * self._fullVel

        if theoryVeloc > self._vel + accel:
            self._vel += accel
        elif theoryVeloc < self._vel - accel:
            self._vel -= accel
        else:
            self._vel += 0.9 * ( theoryVeloc - self._vel )

        current = abs(outPerc) * 30
        statorCurrent = 0 if outPerc ==0 else current / abs(outPerc)

        #print( f"{outPerc}, {accel}, {self.extendVeloc}" )
        self._sim.addQuadraturePosition( int(self._vel * period * 100) )
        self._sim.setQuadratureVelocity( int(self._vel) )
        self._sim.setSupplyCurrent( current )
        self._sim.setStatorCurrent( statorCurrent )
        self._sim.setBusVoltage( 12 - outPerc * outPerc * 3/4 )
        pass

class TalonFxCanSimProfile(SimProfile):
    _simProfile:TalonFxSimProfile
    _sensor:CANCoder
    _sim:CANCoderSimCollection

    def __init__(self, motor:TalonFX, accelTime:float, maxVelocity:float, sensorPhase:bool, sensor:CANCoder):
        self._simProfile = TalonFxSimProfile( motor, accelTime, maxVelocity, sensorPhase )
        self._sensor = sensor
        self._sim = sensor.getSimCollection()

    def run(self, period):
        self._simProfile.run(period)
        self._sim.setVelocity( int( self._simProfile._vel ) )
        self._sim.setRawPosition( int( self._simProfile._motor.getClosedLoopTarget() ) ) #int( self._simProfile._vel * period * 100 ) )
        #self._sim.setRawPosition( int( self._motor.getSelectedSensorPosition() ) )
        pass

class PhysicsEngine:
    _simProfiles:typing.List[SimProfile] = []

    def __init__(self,physics_controller,robot):
        self.physics_controller:PhysicsInterface = physics_controller

        myRobot:Robot = robot
        self.myRobot = myRobot

        # DriveTrain
        self._simProfiles.append( TalonFxSimProfile( myRobot.m_robotContainer.swerveDrive.moduleFL.driveMotor, 1.0, 4150, False ) )
        self._simProfiles.append( TalonFxCanSimProfile( myRobot.m_robotContainer.swerveDrive.moduleFL.angleMotor, 1.0, 4150, False, myRobot.m_robotContainer.swerveDrive.moduleFL.angleSensor ) )

        self._simProfiles.append( TalonFxSimProfile( myRobot.m_robotContainer.swerveDrive.moduleFR.driveMotor, 1.0, 4150, False ) )
        self._simProfiles.append( TalonFxCanSimProfile( myRobot.m_robotContainer.swerveDrive.moduleFR.angleMotor, 1.0, 4150, False, myRobot.m_robotContainer.swerveDrive.moduleFR.angleSensor ) )
        
        self._simProfiles.append( TalonFxSimProfile( myRobot.m_robotContainer.swerveDrive.moduleBL.driveMotor, 1.0, 4150, False ) )
        self._simProfiles.append( TalonFxCanSimProfile( myRobot.m_robotContainer.swerveDrive.moduleBL.angleMotor, 1.0, 4150, False, myRobot.m_robotContainer.swerveDrive.moduleBL.angleSensor ) )
        
        self._simProfiles.append( TalonFxSimProfile( myRobot.m_robotContainer.swerveDrive.moduleBR.driveMotor, 1.0, 4150, False ) )
        self._simProfiles.append( TalonFxCanSimProfile( myRobot.m_robotContainer.swerveDrive.moduleBR.angleMotor, 1.0, 4150, False, myRobot.m_robotContainer.swerveDrive.moduleBR.angleSensor ) )
        
        # Arm Components
        self._simProfiles.append( TalonFxSimProfile( myRobot.m_robotContainer.armPivot.pivotMotor, 1.0, 4150, False ) )
        self._simProfiles.append( TalonSrxSimProfile( myRobot.m_robotContainer.armExtend.extendMotor, 1.0, 4150, False ) )

        self.physics_controller.field.setRobotPose( self.myRobot.m_robotContainer.swerveDrive.getPose() )

    def update_sim(self, now, tm_diff):
        # Don't Update Simulation Environment when Disabled
        if RobotState.isDisabled(): return None

        # Update Each Motor
        for x in self._simProfiles:
            x.run(period=tm_diff)

        speed = self.myRobot.m_robotContainer.swerveDrive.kinematics.toChassisSpeeds(
            self.myRobot.m_robotContainer.swerveDrive.moduleFL.moduleState,
            self.myRobot.m_robotContainer.swerveDrive.moduleFR.moduleState,
            self.myRobot.m_robotContainer.swerveDrive.moduleBL.moduleState,
            self.myRobot.m_robotContainer.swerveDrive.moduleBR.moduleState
        )
        ftSpeed = ChassisSpeeds(vx = speed.vx_fps, vy = speed.vy_fps, omega= speed.omega )

        pose = self.physics_controller.drive( ftSpeed, tm_diff )
        newRotation = pose.rotation()
        self.myRobot.m_robotContainer.swerveDrive.gyro.getSimCollection().setRawHeading( newRotation.degrees() )
        


