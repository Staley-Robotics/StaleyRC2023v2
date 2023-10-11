"""
Description: CTRE Power Distribution Panel
Version:  1
Date:  2023-10-09

Dependencies:
- None

CAN Network:  Default
"""

### Imports
# Python Imports

# FRC Component Imports
from commands2 import SubsystemBase
from wpilib import PowerDistribution

# Our Imports

### Constants

### Class: PowerDistPanel
class PowerDistPanel(SubsystemBase):
    _panel:PowerDistribution = None

    def __init__(self):
        super().__init__()
        self.setSubsystem("PowerDistPanel")
        self.setName("PowerDistPanel")

        self._panel = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE)
        #self.addChild( "PowerDistPanel", self._panel )
