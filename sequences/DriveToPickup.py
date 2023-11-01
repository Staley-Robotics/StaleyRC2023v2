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
from util import *

# Constants
kMaxSpeedMetersPerSecond = 3
kMaxAccelMetersPerSecondSq = 3
kMaxAngularSpeedMetersPerSecond = math.pi
kMaxAngularAccelMetersPerSecondSq = math.pi

class DriveToPickup(ConditionalCommand):
    def __init__(self, swerveDrive:SwerveDrive, nav:Navigation):        
        super().__init__(
            self.DriveToPickupPath(swerveDrive, nav, False),
            self.DriveToPickupPath(swerveDrive, nav, True),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        )
        self.setName( "DriveToPickup" )
        self.addRequirements( swerveDrive )


    class DriveToPickupPath(SequentialCommandGroup):
        __ntTbl__ = NetworkTableInstance.getDefault().getTable( "Navigation" )

        def __init__(self, swerveDrive:SwerveDrive, nav:Navigation, isRedAlliance:bool=False):
            super().__init__( commands=() )
            alliance = "Blue" if not isRedAlliance else "Red"
            self.setName( f"DriveToPickupPath-{alliance}" )
            self.addRequirements( swerveDrive )
            self.DriveTrain = swerveDrive
            self.__nav__ = nav
            self.__isRedAlliance__ = isRedAlliance

            rDropoffRobotMiddle = ChargedUp.getRow("DropoffRobotMiddle")
            rChargeNear = ChargedUp.getRow("ChargeNear")
            rSafeExit = ChargedUp.getRow("SafeExit")
            rArenaMiddle = ChargedUp.getRow("ArenaMiddle")
            rPickupEntry = ChargedUp.getRow("PickupEntry")
            rPickupRobotNormal = ChargedUp.getRow("PickupRobotNormal")
            rPickupEdge = ChargedUp.getRow("PickupEdge")

            cChargeEdgeLeft = ChargedUp.getCol("ChargeEdgeLeft", isRedAlliance)
            cSafeExitLeft = ChargedUp.getCol("SafeExitLeft", isRedAlliance)
            cChargeEdgeRight = ChargedUp.getCol("ChargeEdgeRight", isRedAlliance)
            cSafeExitRight = ChargedUp.getCol("SafeExitRight", isRedAlliance)
            cChargeBalanceLeft = ChargedUp.getCol("ChargeBalanceLeft", isRedAlliance)
            cChargeBalanceRight = ChargedUp.getCol("ChargeBalanceRight", isRedAlliance)
            cPickupDropoffEdge = ChargedUp.getCol("PickupDropoffEdge", isRedAlliance)
            cPickupEntry = ChargedUp.getCol("PickupEntry", isRedAlliance)
            cPickupLeft = ChargedUp.getCol("PickupLeft", isRedAlliance)
            cPickupRight = ChargedUp.getCol("PickupRight", isRedAlliance)

            tALeft = Translation2d( rDropoffRobotMiddle, cChargeEdgeLeft )
            tARight = Translation2d( rDropoffRobotMiddle, cChargeEdgeRight )
            
            tBLeft = Translation2d( rChargeNear, cSafeExitLeft )
            tBRight = Translation2d( rChargeNear, cSafeExitRight )
            
            tCLeft = Translation2d( rSafeExit, cSafeExitLeft )
            tCMiddle = Translation2d( rSafeExit, cChargeBalanceLeft )
            tCRight = Translation2d( rSafeExit, cSafeExitRight)

            tD = Translation2d( rArenaMiddle, cPickupDropoffEdge )
            
            tE = Translation2d( rPickupEntry, cPickupRight )
            
            tFLeft = Translation2d( rPickupRobotNormal, cPickupLeft )
            tFRight = Translation2d( rPickupRobotNormal, cPickupRight )

            dALeft   = DriveToPose( swerveDrive, lambda: Pose2d( tALeft,   swerveDrive.getRobotAngle() ) )
            dARight  = DriveToPose( swerveDrive, lambda: Pose2d( tARight,  swerveDrive.getRobotAngle() ) )
            dBLeft   = DriveToPose( swerveDrive, lambda: Pose2d( tBLeft,   swerveDrive.getRobotAngle() ) )
            dBRight  = DriveToPose( swerveDrive, lambda: Pose2d( tBRight,  swerveDrive.getRobotAngle() ) )
            dCLeft   = DriveToPose( swerveDrive, lambda: Pose2d( tCLeft,   swerveDrive.getRobotAngle() ) )
            dCMiddle = DriveToPose( swerveDrive, lambda: Pose2d( tCMiddle, swerveDrive.getRobotAngle() ) )
            dCRight  = DriveToPose( swerveDrive, lambda: Pose2d( tCRight,  swerveDrive.getRobotAngle() ) )
            dD       = DriveToPose( swerveDrive, lambda: Pose2d( tD,       Rotation2d(0).fromDegrees(-90) ) )
            dE       = DriveToPose( swerveDrive, lambda: Pose2d( tE,       Rotation2d(0).fromDegrees(0) ) )
            dFLeft   = DriveToPose( swerveDrive, lambda: Pose2d( tFLeft,   Rotation2d(0).fromDegrees(0) ) )
            dFRight  = DriveToPose( swerveDrive, lambda: Pose2d( tFRight,  Rotation2d(0).fromDegrees(0) ) )
            
            self.addCommands(
                ConditionalCommand(
                    dALeft,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rChargeNear, yMin=cChargeEdgeLeft, yMax=cChargeEdgeRight ) and self.isExitLeft()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dARight,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rChargeNear, yMin=cChargeEdgeLeft, yMax=cChargeEdgeRight ) and self.isExitRight()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( self.getNearestMe( [tALeft, tARight] ), swerveDrive.getRobotAngle() ) ),
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rChargeNear, yMin=cChargeEdgeLeft, yMax=cChargeEdgeRight )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( self.getNearestMe( [tBLeft, tBRight] ), swerveDrive.getRobotAngle() ) ),
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rChargeNear )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( self.getNearestMe( [tCLeft, tCMiddle, tCRight] ), swerveDrive.getRobotAngle() ) ),
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rSafeExit )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dD,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rArenaMiddle )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dE,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rPickupEntry )
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dFLeft,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rPickupEdge ) and self.isPickupLeft()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    dFRight,
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rPickupEdge ) and self.isPickupRight()
                )
            )
            self.addCommands(
                ConditionalCommand(
                    DriveToPose( swerveDrive, lambda: Pose2d( self.getNearestMe( [tFLeft, tFRight] ), Rotation2d(0).fromDegrees(0) ) ),
                    commands2.cmd.nothing(),
                    lambda: self.isPoseValid( xMax=rPickupEdge )
                )
            )
            

        def isExitLeft(self) -> bool:
            exit = self.__ntTbl__.getString( "ExitZone", "None" )
            value = "Left" if not self.__isRedAlliance__ else "Right"
            return exit == value
        
        def isExitMiddle(self) -> bool:
            exit = self.__ntTbl__.getString( "ExitZone", "None" )
            return exit == "Middle"
        
        def isExitRight(self) -> bool:
            exit = self.__ntTbl__.getString( "ExitZone", "None" )
            value = "Right" if not self.__isRedAlliance__ else "Left"
            return exit == value

        def isPickupLeft(self) -> bool:
            exit = self.__ntTbl__.getString( "PickupZone", "None" )
            value = "Left" if not self.__isRedAlliance__ else "Right"
            return exit == value
        
        def isPickupRight(self) -> bool:
            exit = self.__ntTbl__.getString( "PickupZone", "None" )
            value = "Right" if not self.__isRedAlliance__ else "Left"
            return exit == value
        
        def getNearestMe(self, translations:list[Translation2d]) -> Translation2d:
            me = self.DriveTrain.getPose().translation()
            goto = me.nearest( translations )
            return goto
        
        def getNearestGoal(self, translations:list[Translation2d]) -> Translation2d:
            isRed = DriverStation.getAlliance() == DriverStation.Alliance.kRed
            column = self.__nav__.getZone("Community").getSelectedValue()
            goal = ChargedUp.getTranslation( "DropoffRobotNormal", column, isRed)
            goto = goal.nearest( translations )
            return goto

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