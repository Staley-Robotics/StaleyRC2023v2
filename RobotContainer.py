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
from autonomous.LeftBasic import LeftBasic
from autonomous.RightBasic import RightBasic

# Define Robot Container Class
class RobotContainer:
    # Initialization
    def __init__(self):
        # Controllers
        self.driver1 = commands2.button.CommandXboxController(0)
        self.driver2 = commands2.button.CommandXboxController(1)

        # Subsystems
        self.swerveDrive = SwerveDrive()
        self.armPivot    = ArmPivot()
        self.armExtend   = ArmExtend()
        self.claw        = Claw()
        self.limelight1  = Limelight( "limelight-one", self.swerveDrive.getOdometry )
        self.limelight2  = Limelight( "limelight-two", self.swerveDrive.getOdometry )
        self.navigation  = Navigation( self.driver1.getPOV, self.driver2.getPOV )

        # SmartDashboard Subsystems
        SmartDashboard.putData(key="SwerveDrive", data=self.swerveDrive)
        SmartDashboard.putData(key="ArmPivot",    data=self.armPivot)
        SmartDashboard.putData(key="ArmExtend",   data=self.armExtend)
        SmartDashboard.putData(key="Claw",        data=self.claw)
        SmartDashboard.putData(key="Limelight1",  data=self.limelight1)
        SmartDashboard.putData(key="Limelight2",  data=self.limelight2)
        SmartDashboard.putData(key="Navigation",  data=self.navigation)

        # SmartDashboard Commands
        SmartDashboard.putData(key="Lockdown",    data=DriveLockdown(self.swerveDrive))
        SmartDashboard.putData(key="ExtendReset", data=ArmExtendReset(self.armExtend))
        SmartDashboard.putData(key="Pickup",      data=DriveToPickup(self.swerveDrive))
        SmartDashboard.putData(key="Dropoff",     data=DriveToDropoff(self.swerveDrive))

        # Configure Default Commands
        self.configureDefaultCommands()

        # Configure Button Bindings
        self.configureButtonBindings()

        # Autonomous Choices
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption(name="None", object=commands2.cmd.nothing())
        self.m_chooser.addOption(name="Sample 1", object=AutoSample1(self.swerveDrive, self.armPivot, self.claw))
        self.m_chooser.addOption(name="Blue Left Place and Exit", object=LeftBasic(self.swerveDrive, self.armPivot, self.armExtend, self.claw))
        self.m_chooser.addOption(name="Blue Right Place and Exit", object=RightBasic(self.swerveDrive, self.armPivot, self.armExtend, self.claw))
        self.m_chooser.addOption(name="Red Left Place and Exit", object=RightBasic(self.swerveDrive, self.armPivot, self.armExtend, self.claw, True))
        self.m_chooser.addOption(name="Red Right Place and Exit", object=LeftBasic(self.swerveDrive, self.armPivot, self.armExtend, self.claw, True))
        SmartDashboard.putData(key="Autonomous Mode", data=self.m_chooser) # Add Autonomous Chooser to Dashboard

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


    def configureButtonBindings(self):
        #### Example: https://github.com/robotpy/examples/blob/main/commands-v2/hatchbot-inlined/robotcontainer.py
        # Driver 1
        self.getDriver1().start().toggleOnTrue( ToggleFieldRelative(self.swerveDrive) )
        self.getDriver1().back().toggleOnTrue( ToggleMotionMagic(self.swerveDrive) )
        self.getDriver1().rightBumper().toggleOnTrue( ToggleHalfSpeed(self.swerveDrive) )
        #self.getDriver1().leftBumper().toggleOnTrue( DriveToPose(self.swerveDrive, lambda: Pose2d( Translation2d(2.0, 3), Rotation2d(0).fromDegrees(180))) )
        self.getDriver1().leftBumper().toggleOnTrue( NavigationToggleZone(self.navigation) )
        self.getDriver1().X().whileTrue( DriveToPickup(self.swerveDrive) )
        self.getDriver1().Y().whileTrue( DriveToDropoff(self.swerveDrive) )

        commands2.button.POVButton( self.getDriver1(),   0 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver1(), 180 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver1(), 270 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver1(),  90 ).whenPressed( commands2.cmd.nothing() )

        # Driver 2
        self.getDriver2().A().toggleOnTrue( ClawAction(self.claw, ClawAction.Action.kToggle) )
        commands2.button.POVButton( self.getDriver2(),   0 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver2(), 180 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver2(), 270 ).whenPressed( commands2.cmd.nothing() )
        commands2.button.POVButton( self.getDriver2(),  90 ).whenPressed( commands2.cmd.nothing() )

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
#    @staticmethod
#    def getInstance(): # -> RobotContainer:
#        return RobotContainer()
