"""
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
"""

from wpilib import DriverStation, PowerDistribution, Timer

"""
/**
 * Add your docs here.
 */
"""
class MotorUtils:
    DEFAULT_TIMEOUT:float = 0.15
    timeout:float
    time:float
    powerDistPanel:PowerDistribution

    PDPChannel:int
    currentThreshold:float

    everStalled:bool = False

    def __init__(self, PDPPort:int, currentThreshold:float, timeout:float, powerDistPanel:PowerDistribution):
        self.timeout = timeout
        self.PDPChannel = PDPPort
        self.currentThreshold = currentThreshold
        self.powerDistPanel = powerDistPanel
        self.time = Timer.getFPGATimestamp()

    def isStalled(self) -> bool:
        currentValue:float = self.powerDistPanel.getCurrent(self.PDPChannel)
        now:float = Timer.getFPGATimestamp()

        if currentValue < self.currentThreshold:
            self.time = now
        elif now - self.time > self.timeout:
            self.everStalled = True
            return True
        else:
            pass

        return False

    def hasStalled(self) -> bool:
        return self.everStalled

    def resetStall(self) -> None:
        self.everStalled = False
