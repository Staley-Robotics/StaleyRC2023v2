# Import Core Libraries
from hal import tResourceType, tInstances
import hal
from wpilib import TimedRobot, run
from commands2 import Command, CommandScheduler


# Import Robot Libraries (Subsystems and Commands)
from RobotContainer import *

# TimedRobot Class
class Robot(TimedRobot):
    m_autonomousCommand:Command = None
    m_robotContainer:RobotContainer = None

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        hal.report( tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder )

    # Run this command every 20 ms
    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    # Autonomous Robot Functions
    def autonomousInit(self):
        self.m_autonomousCommand = self.m_robotContainer.getAutonomousCommand()

        if ( self.m_autonomousCommand != None ):
            self.m_autonomousCommand.schedule()
        
    def autonomousPeriodic(self): pass
    def autonomousExit(self):
        self.m_autonomousCommand.cancel()

    # Teleop Robot Functions
    def teleopInit(self): pass
    def teleopPeriodic(self): pass
    def teleopExit(self): pass

    # Test Robot Functions
    def testInit(self): pass
    def testPeriodic(self): pass
    def testExit(self): pass

    # Disabled Robot Functions
    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

# Start the Robot when Executing Code
if __name__ == '__main__':
    run(Robot)