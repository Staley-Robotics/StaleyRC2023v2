# Import: WPI Libraries
from wpilib import XboxController, SendableChooser, SmartDashboard
import commands2.button
import commands2.cmd
from commands2 import * #Command 

# Import: Subsystems and Commands
from subsystems import *
from commands import *
from sequences import *
from autonomous.AutoSample1 import AutoSample1

# Define Robot Container Class
class RobotContainer:

    # Commands
    cmds = dict()

    def __init__(self):
        # Subsystems
        self.swerveDrive:SwerveDrive = SwerveDrive()
        #pneumatics = Pneumatics()
        self.armPivot = ArmPivot()
        self.armExtend = ArmExtend()
        self.claw = Claw()
        #self.pdp = PowerDistPanel()
        self.limelight1 = Limelight( "limelight-one", self.swerveDrive.getOdometry )
        self.limelight2 = Limelight( "limelight-two", self.swerveDrive.getOdometry )

        # Controllers
        self.driver1 = commands2.button.CommandXboxController(0)
        self.driver2 = commands2.button.CommandXboxController(1)

        # List of Commands
        self.cmds = dict( {
            'auto1': AutoSample1(self.swerveDrive, self.armPivot, self.claw),
            'lockdown': DriveLockdown(self.swerveDrive),
            'clawToggle': ClawAction(self.claw, ClawAction.Action.kToggle),
            'halfspeed': ToggleHalfSpeed(self.swerveDrive),
            'fieldrelative': ToggleFieldRelative(self.swerveDrive),
            'motionmagic': ToggleMotionMagic(self.swerveDrive),
            'extendreset': ArmExtendReset(self.armExtend) #,
            #'arm-pivot-max': ArmPivotPosition(self.armPivot, lambda: ArmPivotPosition.Direction.kMax),
            #'arm-pivot-min': ArmPivotPosition(self.armPivot, lambda: ArmPivotPosition.Direction.kMin),
            #'arm-pivot-up': ArmPivotPosition(self.armPivot, lambda: ArmPivotPosition.Direction.kNext),
            #'arm-pivot-down': ArmPivotPosition(self.armPivot, lambda: ArmPivotPosition.Direction.kPrev) 
        } )

        # SmartDashboard Subsystems
        SmartDashboard.putData(key="SwerveDrive", data=self.swerveDrive)
        #SmartDashboard.putData(key="Pneumatics",  data=self.pneumatics)
        SmartDashboard.putData(key="ArmPivot",    data=self.armPivot)
        SmartDashboard.putData(key="ArmExtend",   data=self.armExtend)
        SmartDashboard.putData(key="Claw",        data=self.claw)
        SmartDashboard.putData(key="Limelight1",  data=self.limelight1)
        SmartDashboard.putData(key="Limelight2",  data=self.limelight2)

        # SmartDashboard Commands
        SmartDashboard.putData(key="Lockdown", data=self.cmds['lockdown']) #DriveLockdown(self.swerveDrive))
        #SmartDashboard.putData(key="ArmPivot-Max", data=self.cmds['arm-pivot-max'])
        #SmartDashboard.putData(key="ArmPivot-Min", data=self.cmds['arm-pivot-min'])
        #SmartDashboard.putData(key="ArmPivot-Up", data=self.cmds['arm-pivot-up'])
        #SmartDashboard.putData(key="ArmPivot-Down", data=self.cmds['arm-pivot-down'])
        SmartDashboard.putData(key="ExtendReset", data=self.cmds['extendreset'])
        SmartDashboard.putData(key="Face Home", data=DriveToRotation(self.swerveDrive, lambda: 180.0))
        SmartDashboard.putData(key="Face Away", data=DriveToRotation(self.swerveDrive, lambda: 0.0))
        SmartDashboard.putData(key="To Pickup", data=DriveToPickup(self.swerveDrive))

        # Configure Default Commands
        self.configureDefaultCommands()

        # Configure Button Bindings
        self.configureButtonBindings()

        # Autonomous Choices
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption(name="None", object=commands2.cmd.nothing())
        self.m_chooser.addOption(name="Sample 1", object=self.cmds['auto1'])
        SmartDashboard.putData(key="Auto Mode", data=self.m_chooser) # Add Autonomous Chooser to Dashboard


    def configureDefaultCommands(self):
        # Drive by Joystick
        self.swerveDrive.setDefaultCommand(
            DriveByStick(
                self.swerveDrive,
                velocityX = lambda: -self.getDriver1().getLeftY(),
                velocityY = lambda: -self.getDriver1().getLeftX(),
                holonomicX = lambda: -self.getDriver1().getRightY(),
                holonomicY = lambda: -self.getDriver1().getRightX(),
                rotate = lambda: self.getDriver1().getLeftTriggerAxis() - self.getDriver1().getRightTriggerAxis() #-self.getDriver1().getRightX()
            )
        )

        # Arm by Joystick
        self.armPivot.setDefaultCommand(
            ArmPivotByStick(
                self.armPivot,
                lambda: -self.getDriver2().getLeftY()
            )
        )
        self.armExtend.setDefaultCommand(
            ArmExtendByStick(
                self.armExtend,
                lambda: -self.getDriver2().getRightY()
            )
        )
        #self.game.setDefaultCommand(
        #    SelectDropoff(
        #        self.game,
        #        lambda: self.getDriver1().getPOV()
        #    )
        #)


    def configureButtonBindings(self):
        #### Example: https://github.com/robotpy/examples/blob/main/commands-v2/hatchbot-inlined/robotcontainer.py
        # Driver 1
        a = self.getDriver1().start().toggleOnTrue( self.cmds['fieldrelative'] )
        self.getDriver1().rightBumper().toggleOnTrue( self.cmds['halfspeed'] )
        self.getDriver1().leftBumper().toggleOnTrue( DriveToPose(self.swerveDrive, lambda: Pose2d( Translation2d(8, 2), Rotation2d(0).fromDegrees(180))) )
        self.getDriver1().back().toggleOnTrue( self.cmds['motionmagic'] )
        commands2.button.POVButton( self.getDriver1(),   0 ).whenPressed( DriveToRotation(self.swerveDrive, lambda: 0.0) )
        commands2.button.POVButton( self.getDriver1(),  90 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 1:  90") ) )
        commands2.button.POVButton( self.getDriver1(), 180 ).whenPressed( DriveToRotation(self.swerveDrive, lambda: 180.0) )
        commands2.button.POVButton( self.getDriver1(), 270 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 1: 270") ) )

        # Driver 2
        self.getDriver2().A().toggleOnTrue( self.cmds['clawToggle'] )
        commands2.button.POVButton( self.getDriver2(),   0 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 2:   0") ) )
        commands2.button.POVButton( self.getDriver2(),  90 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 2:  90") ) )
        commands2.button.POVButton( self.getDriver2(), 180 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 2: 180") ) )
        commands2.button.POVButton( self.getDriver2(), 270 ).whileTrue( commands2.cmd.runOnce( lambda: print("Controller 2: 270") ) )

    # Return Controllers
    def getDriver1(self) -> commands2.button.CommandXboxController:
        return self.driver1
    
    def getDriver2(self) -> commands2.button.CommandXboxController:
        return self.driver2
    
    # Return Autonomous Command
    def getAutonomousCommand(self) -> Command:
        return self.m_chooser.getSelected()

# Create RobotContainer when importing package
#m_robotContainer = RobotContainer()

# Return the RobotContainer Object
#    def getInstance(self):# -> RobotContainer:
#        return self
