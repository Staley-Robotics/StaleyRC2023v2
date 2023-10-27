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
from wpilib import DoubleSolenoid, PneumaticsModuleType

# Our Imports

### Constants

### Class: Pneumatics
class Claw(SubsystemBase):
    # Public Variables

    # Constructor
    def __init__(self):
        super().__init__()
        self.solenoid = DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 3)

        self.setSubsystem( "Claw" )
        self.setName( "Claw" )
        self.addChild( "Solenoid", self.solenoid )

    # Periodic
    def periodic(self):
        # Safeguard to solenoid in Off State
        if self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set( DoubleSolenoid.Value.kReverse )

    ### Claw Function Commands
    # Open Claw
    def open(self):
        self.solenoid.set( DoubleSolenoid.Value.kForward )

    # Close Claw
    def close(self):
        self.solenoid.set( DoubleSolenoid.Value.kReverse )

    # Toggle Claw
    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set( DoubleSolenoid.Value.kReverse )
        if self.isDisabled():
            self.close()
        else:
            self.solenoid.toggle()

    ### Claw State Functions
    # Is Open?
    def isOpen(self) -> bool:
        return self.solenoid.get() == DoubleSolenoid.Value.kForward

    # Is Closed?
    def isClosed(self) -> bool:
        return self.solenoid.get() == DoubleSolenoid.Value.kReverse
    
    # Is Disabled?
    def isDisabled(self) -> bool:
        return self.solenoid.get() == DoubleSolenoid.Value.kOff
