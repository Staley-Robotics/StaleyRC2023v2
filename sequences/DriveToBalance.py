# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

class DriveToBalance(SequentialCommandGroup):
    def __init__(self, swerveDrive:SwerveDrive):
        super().__init__( commands=() )
        self.setName( "Charge" )
        self.addRequirements( swerveDrive )

        #self.addCommands()
        
