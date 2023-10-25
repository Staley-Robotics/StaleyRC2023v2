# Import Python

# Import FRC
from commands2 import InstantCommand, CommandBase
from wpilib import RobotState

# Import Subsystems and Commands
from subsystems import SwerveDrive

# Constants


# Toggle: Half Speed
class ToggleHalfSpeed(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.halfSpeed.toggle()
        )

    def runsWhenDisabled(self) -> bool: return True

# Toggle: Field Relative
class ToggleFieldRelative(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.fieldRelative.toggle()
        )
    
    def runsWhenDisabled(self) -> bool: return True

# Toggle: Motion Magic
class ToggleMotionMagic(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.motionMagic.toggle()
        )
    
    def runsWhenDisabled(self) -> bool: return True
