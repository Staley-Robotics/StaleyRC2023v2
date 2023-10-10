"""
Description: CTRE Pneumatics Compressor Control System
Version:  1
Date:  2023-10-03

Dependencies:
- None

CAN Network:  Default
"""

### Imports
# Python Imports

# FRC Component Imports
from commands2 import SubsystemBase
from wpilib import PneumaticsControlModule, Compressor

# Our Imports

### Constants

### Class: Pneumatics
class Pneumatics(SubsystemBase):
    # Public Variables
    module:PneumaticsControlModule = None
    compressor:Compressor = None


    # Constructor
    def __init__(self):
        super().__init__()
        self.module = PneumaticsControlModule(0)
        self.module.clearAllStickyFaults()
        self.compressor = self.module.makeCompressor()
        self.compressor.disable()

        self.setSubsystem( "Pneumatics" )
        self.setName( "Pneumatics" )
        self.addChild( "Compressor", self.compressor )


    # Run on all iterations of TimedRobot periodic commands
    def periodic(self):
        # Get Compressor States
        isEnabled:bool = self.compressor.isEnabled()
        atPressure:bool = not self.compressor.getPressureSwitchValue()
        
        # Set Compressor Run Mode
        if isEnabled and atPressure:
            self.compressor.disable()
        if not isEnabled and not atPressure:
            self.compressor.enableDigital()
