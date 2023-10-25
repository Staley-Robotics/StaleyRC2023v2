# Import Python
import math

# Import FRC
from commands2 import SequentialCommandGroup
import commands2.cmd
from wpilib import DriverStation
from wpimath.geometry import Translation2d

# Import Subsystems and Commands
from subsystems import *
from commands import *

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

class DriveToDropoff(ConditionalCommand):
    def __init__(self, swerveDrive:SwerveDrive):        
        super().__init__(
            self.DriveToDropoffPath(swerveDrive, False),
            self.DriveToDropoffPath(swerveDrive, True),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        )
        self.setName( "DriveToDropoff" )
        self.addRequirements( swerveDrive )


    class DriveToDropoffPath(SequentialCommandGroup):
        __ntTbl__ = NetworkTableInstance.getDefault().getTable("Navigation")

        def __init__(self, swerveDrive:SwerveDrive, isRedAlliance:bool=False):
            super().__init__( commands=() )
            alliance = "Blue" if not isRedAlliance else "Red"
            self.setName( f"DriveToDropoffPath-{alliance}" )
            self.addRequirements( swerveDrive )
            self.DriveTrain = swerveDrive

            rDropoffRobotMiddle = ChargedUp.getRow("DropoffRobotMiddle")
            rChargeNear = ChargedUp.getRow("ChargeNear")
            rChargeFar = ChargedUp.getRow("ChargeFar")
            rSafeExit = ChargedUp.getRow("SafeExit")
            rGroundObject= ChargedUp.getRow("GroundObject")
            rArenaMiddle = ChargedUp.getRow("ArenaMiddle")
            rPickupEntry = ChargedUp.getRow("PickupEntry")
            rPickupRobotNormal = ChargedUp.getRow("PickupRobotNormal")
            rPickupEdge = ChargedUp.getRow("PickupEdge")

            cChargeEdgeLeft = ChargedUp.getCol("ChargeEdgeLeft", isRedAlliance)
            cSafeExitLeft = ChargedUp.getCol("SafeExitLeft", isRedAlliance)
            cChargeEdgeRight = ChargedUp.getCol("ChargeEdgeRight", isRedAlliance)
            cSafeExitRight = ChargedUp.getCol("SafeExitRight", isRedAlliance)
            cChargeBalanceLeft = ChargedUp.getCol("ChargeBalanceLeft", isRedAlliance)
            cChargeBalanceMiddle = ChargedUp.getCol("ChargeBalanceMiddle", isRedAlliance)
            cChargeBalanceRight = ChargedUp.getCol("ChargeBalanceRight", isRedAlliance)
            cPickupDropoffEdge = ChargedUp.getCol("PickupDropoffEdge", isRedAlliance)
            cPickupEntry = ChargedUp.getCol("PickupEntry", isRedAlliance)
            cPickupLeft = ChargedUp.getCol("PickupLeft", isRedAlliance)
            cPickupRight = ChargedUp.getCol("PickupRight", isRedAlliance)

            tA = Translation2d( rPickupEntry, cPickupRight )

            tBLeft = Translation2d( rSafeExit, cSafeExitLeft )
            tBMiddle = Translation2d( rSafeExit, cChargeBalanceMiddle )
            tBRight = Translation2d( rSafeExit, cSafeExitRight )

            tCLeft = Translation2d( rDropoffRobotMiddle, cSafeExitLeft )
            tCMiddle = Translation2d( rDropoffRobotMiddle, cChargeBalanceMiddle )
            tCRight = Translation2d( rDropoffRobotMiddle, cSafeExitRight )

            dA       = DriveToPose( swerveDrive, lambda: Pose2d( tA,       Rotation2d(0).fromDegrees(-90) ) )
            dBLeft   = DriveToPose( swerveDrive, lambda: Pose2d( tBLeft,   Rotation2d(0).fromDegrees(180) ) )
            dBMiddle = DriveToPose( swerveDrive, lambda: Pose2d( tBMiddle, Rotation2d(0).fromDegrees(180) ) )
            dBRight  = DriveToPose( swerveDrive, lambda: Pose2d( tBRight,  Rotation2d(0).fromDegrees(180) ) )
            dBRight2 = DriveToPose( swerveDrive, lambda: Pose2d( tBRight,  Rotation2d(0).fromDegrees(180) ) )
            dCLeft   = DriveToPose( swerveDrive, lambda: Pose2d( tCLeft,   Rotation2d(0).fromDegrees(180) ) )
            dCMiddle = DriveToPose( swerveDrive, lambda: Pose2d( tCMiddle, Rotation2d(0).fromDegrees(180) ) )
            dCRight  = DriveToPose( swerveDrive, lambda: Pose2d( tCRight, Rotation2d(0).fromDegrees(180) ) )
            #dD       = DriveToPose( swerveDrive, lambda: Pose2d( tCRight,  Rotation2d(0).fromDegrees(0) ) )
            
            self.addCommands(
                ConditionalCommand(
                    dA,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rPickupEntry )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dBLeft,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rSafeExit, yMin=cChargeBalanceMiddle ) and self.isEnterLeft()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dBMiddle,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rChargeNear ) and self.isEnterMiddle()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dBRight,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rSafeExit ) and self.isEnterRight()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dBRight2,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rSafeExit, yMax=cChargeBalanceMiddle )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dCLeft,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rDropoffRobotMiddle, yMin=cChargeBalanceLeft, yMax=cPickupRight )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dCMiddle,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rDropoffRobotMiddle, xMax=rChargeFar, yMin=cChargeEdgeRight, yMax=cChargeEdgeLeft )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dCRight,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMin=rDropoffRobotMiddle, yMax=cChargeBalanceRight )
                )
            )
            

        def isEnterLeft(self) -> bool:
            exit = self.__ntTbl__.getString( "EntryZone", "None" )
            return exit == "Left"
        
        def isEnterMiddle(self) -> bool:
            exit = self.__ntTbl__.getString( "EntryZone", "None" )
            return exit == "Middle"
        
        def isEnterRight(self) -> bool:
            exit = self.__ntTbl__.getString( "EntryZone", "None" )
            return exit == "Right"
        
        def isPoseValid( self,
                         xMin:float = None,
                         xMax:float = None,
                         yMin:float = None,
                         yMax:float = None
                        ) -> bool:
            # Valid unless otherwise not
            valid = True
            
            # Get Pose
            x = self.DriveTrain.getPose().X()
            y = self.DriveTrain.getPose().Y()
            
            # Reverse Y variables if Red Alliance
            if DriverStation.getAlliance == DriverStation.Alliance.kRed:
                temp = yMin
                yMin = yMax
                yMax = temp

            # Is X Valid?
            if valid and xMin != None:
                valid = x >= xMin
            if valid and xMax != None:
                valid = x <= xMax

            # Is Y Valid?
            if valid and yMin != None:
                valid = y >= yMin
            if valid and yMax != None:
                valid = y <= yMax

            # Return Valid Results
            return valid