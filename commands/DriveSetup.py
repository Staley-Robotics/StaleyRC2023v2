# Import Python

# Import FRC
from commands2 import InstantCommand

# Import Subsystems and Commands
from subsystems import SwerveDrive

# Constants


# Toggle: Half Speed
class ToggleHalfSpeed(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.halfspeedToggle()
        )

    def runsWhenDisabled(self) -> bool: return True

# Toggle: Field Relative
class ToggleFieldRelative(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.fieldrelativeToggle()
        )
    
    def runsWhenDisabled(self) -> bool: return True

# Toggle: Motion Magic
class ToggleMotionMagic(InstantCommand):
    def __init__(self, DriveSubsystem:SwerveDrive) -> None:
        super().__init__(
            toRun=lambda: DriveSubsystem.motionmagicToggle()
        )
    
    def runsWhenDisabled(self) -> bool: return True
