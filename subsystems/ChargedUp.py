import typing
from commands2 import *
from wpilib import DriverStation, Field2d, FieldObject2d, SmartDashboard
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

class ChargedUp:
    rows = dict(
        ArenaNear = 0,
        DropoffTop = 0.385,
        DropoffMiddle = 0.817,
        DropoffBottom = 1.175,
        DropoffEdge = 1.425,
        DropoffRobotEdge = 1.831,
        DropoffRobotNormal = 2.034,
        DropoffRobotMiddle = 2.175,
        ChargeNear = 2.925,
        ChargeCenter = 3.9075,
        ChargeFar = 4.89,
        SafeExit = 5.296,
        GroundObject = 7.075,
        ArenaMiddle = 8.2615,
        PickupEntry = 11.633, #12.75,
        PickupRobot = 14.225,
        PickupRobotNormal = 15.576,
        PickupRobotEdge = 15.779,
        PickupEdge = 16.185,
        ArenaFar = 16.523
    )
    columns = dict(
        ArenaLeft = 8.013,
        PickupSide = 7.607,
        PickupLeft = 7.35,
        PickupEntry = 6.775,
        PickupRight = 6.2,
        PickupDropoffEdge = 5.475,
        Dropoff1 = 4.98,
        SafeExitLeft = 4.725,
        GroundObject1 = 4.575,
        Dropoff2 = 4.425,
        ChargeEdgeLeft = 3.975,
        Dropoff3 = 3.865,
        ChargeBalanceLeft = 3.525,
        GroundObject2 = 3.365,
        Dropoff4 = 3.305,
        ChargeBalanceMiddle = 2.75,
        Dropoff5 = 2.735,
        Dropoff6 = 2.18,
        GroundObject3 = 2.135,
        ChargeBalanceRight = 1.95,
        Dropoff7 = 1.625,
        ChargeEdgeRight = 1.525,
        Dropoff8 = 1.075,
        GroundObject4 = 0.925,
        SafeExitRight = 0.7625,
        Dropoff9 = 0.5,
        ArenaRight = 0
    )

    @staticmethod
    def getRow(rowName):
        return ChargedUp.rows[rowName]
    
    @staticmethod
    def getCol(columnName:str, isRedAlliance:bool=False) -> float:
        y = ChargedUp.columns[columnName]
        if isRedAlliance: y = ChargedUp.columns["ArenaLeft"] - y
        return y
    
    def getPose(rowName, columnName, degrees, isRed:bool=False):
        x = ChargedUp.getRow(rowName)
        y = ChargedUp.getCol(columnName, isRed)
        pose = Pose2d( x, y, Rotation2d().fromDegrees( degrees ) )
        return pose
