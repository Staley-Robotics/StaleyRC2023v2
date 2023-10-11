# Import: WPI Libraries
from wpilib import XboxController, SendableChooser, SmartDashboard
import commands2.button
import commands2.cmd
from commands2 import Command 

# Import: Subsystems and Commands
from subsystems import *
from commands import *
from sequences import *
from autonomous.AutoSample1 import AutoSample1

# Define Robot Container Class
class RobotContainer:
    
    # Subsystems
    swerveDrive = SwerveDrive()
    #pneumatics = Pneumatics()
    armPivot = ArmPivot()
    armExtend = ArmExtend()
    claw = Claw()
    pdp = PowerDistPanel()

    # Controllers
    driver1 = commands2.button.CommandXboxController(0)
    driver2 = commands2.button.CommandXboxController(1)

    # Commands
    cmds = dict()

    def __init__(self):
        # List of Commands
        self.cmds = dict( {
            'auto1': AutoSample1(self.swerveDrive, self.armPivot, self.claw),
            'scurve': SCurve(self.swerveDrive),
            'lockdown': DriveLockdown(self.swerveDrive),
            'clawToggle': ClawAction(self.claw, ClawAction.Action.kToggle),
            'halfspeed': ToggleHalfSpeed(self.swerveDrive),
            'fieldrelative': ToggleFieldRelative(self.swerveDrive),
            'motionmagic': ToggleMotionMagic(self.swerveDrive)#,
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

        # SmartDashboard Commands
        SmartDashboard.putData(key="SCurve", data=self.cmds['scurve']) #SCurve(self.swerveDrive)
        SmartDashboard.putData(key="Lockdown", data=self.cmds['lockdown']) #DriveLockdown(self.swerveDrive))
        #SmartDashboard.putData(key="ArmPivot-Max", data=self.cmds['arm-pivot-max'])
        #SmartDashboard.putData(key="ArmPivot-Min", data=self.cmds['arm-pivot-min'])
        #SmartDashboard.putData(key="ArmPivot-Up", data=self.cmds['arm-pivot-up'])
        #SmartDashboard.putData(key="ArmPivot-Down", data=self.cmds['arm-pivot-down'])

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
                lambda: -self.getDriver1().getLeftY(),
                lambda: -self.getDriver1().getLeftX(),
                lambda: -self.getDriver1().getRightY(),
                lambda: -self.getDriver1().getRightX(),
                lambda: self.swerveDrive.halfSpeed.get()
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


    def configureButtonBindings(self):
        #### Example: https://github.com/robotpy/examples/blob/main/commands-v2/hatchbot-inlined/robotcontainer.py
        # Driver 1
        self.getDriver1().start().toggleOnTrue( self.cmds['fieldrelative'] )
        self.getDriver1().rightBumper().toggleOnTrue( self.cmds['halfspeed'] )
        self.getDriver1().leftBumper().whileTrue( self.cmds['scurve'] )
        self.getDriver1().back().toggleOnTrue( self.cmds['motionmagic'])
        
        # Driver 2
        self.getDriver2().rightBumper().toggleOnTrue( self.cmds['clawToggle'] )

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
