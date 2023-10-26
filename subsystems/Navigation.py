import typing

from commands2 import *
from wpilib import *
from wpimath.geometry import *
from ntcore import *
# from subsystems import ChargedUp

class Navigation(SubsystemBase):
    def __init__(self, con1getPov: lambda: -1, con2getPov: lambda: -1 ) -> None:
        super().__init__()

        self.__con1getPov__ = con1getPov
        self.__con2getPov__ = con2getPov

        self.__pickup__ = NavigationZone( "PickupZone" )
        self.__pickup__.addOption( "Nearest", "None", "None" )
        self.__pickup__.addOption( "Left", "PickupLeft", "PickupRight" )
        self.__pickup__.addOption( "Right", "PickupRight", "PickupLeft" )

        self.__entryzone__ = NavigationZone( "EntryZone" )
        self.__entryzone__.addOption( "Nearest", "None", "None" )
        self.__entryzone__.addOption( "Left", "SafeExitLeft", "SafeExitRight" )
        #self.__entryzone__.addOption( "Middle", "ChargeBalanceMiddle", "ChargeBalanceMiddle" )
        self.__entryzone__.addOption( "Right", "SafeExitRight", "SafeExitLeft" )

        self.__exitzone__ = NavigationZone( "ExitZone" )
        self.__exitzone__.addOption( "Nearest", "None", "None" )
        self.__exitzone__.addOption( "Left", "SafeExitLeft", "SafeExitLeft" )
        #self.__exitzone__.addOption( "Middle", "ChargeBalanceLeft", "ChargeBalanceRight" )
        self.__exitzone__.addOption( "Right", "SafeExitRight", "SafeExitLeft" )

        #self.__ground__ = NavigationZone( "GroundZone")
        #self.__ground__.addOption( "Nearest", "None", "None" )
        #self.__ground__.addOption( "FarLeft", "GroundObject1", "GroundObject4" )
        #self.__ground__.addOption( "MiddleLeft", "GroundObject2", "GroundObject3" )
        #self.__ground__.addOption( "MiddleRight", "GroundObject3", "GroundObject2" )
        #self.__ground__.addOption( "FarRight", "GroundObject4", "GroundObject1" )

        self.__balance__ = NavigationZone( "BalanceZone" )
        self.__balance__.addOption( "Nearest", "None", "None" )
        self.__balance__.addOption( "Left", "ChargeBalanceLeft", "ChargeBalanceRight" )
        self.__balance__.addOption( "Middle", "ChargeBalanceMiddle", "ChargeBalanceMiddle" )
        self.__balance__.addOption( "Right", "ChargeBalanceRight", "ChargeBalanceLeft" )

        self.__scoreRow__ = NavigationZone( "ScoreRow" )
        self.__scoreRow__.addOption( "Top", "DropoffTop", "DropoffTop" )
        self.__scoreRow__.addOption( "Middle", "DropoffMiddle", "DropoffMiddle" )
        self.__scoreRow__.addOption( "Bottom", "DropoffBottom", "DropoffBottom" )

        self.__community__ = NavigationZone( "Community" )
        self.__community__.addOption( "A", "Dropoff1", "Dropoff9" )
        self.__community__.addOption( "B", "Dropoff2", "Dropoff8" )
        self.__community__.addOption( "C", "Dropoff3", "Dropoff7" )
        self.__community__.addOption( "D", "Dropoff4", "Dropoff6" )
        self.__community__.addOption( "E", "Dropoff5", "Dropoff5" )
        self.__community__.addOption( "F", "Dropoff6", "Dropoff4" )
        self.__community__.addOption( "G", "Dropoff7", "Dropoff3" )
        self.__community__.addOption( "H", "Dropoff8", "Dropoff2" )
        self.__community__.addOption( "I", "Dropoff9", "Dropoff1" )

        self.__scoreGrid__ = NavigationZoneGrid(
            "ScoreZone",
            self.__scoreRow__,
            self.__community__
        )

        self.setDefaultCommand( NavigationUpdateZone(self, "ScoreZone") )

    def getPOV(self):
        # Get Controller POVs
        pov1 = self.__con1getPov__()
        pov2 = self.__con2getPov__()

        # If both drivers provide value, take Driver 1
        if pov1 != -1 and pov2 != -1:
            pov = pov1
        else:
            pov = max( pov1, pov2 )

        # Return Value
        return pov

    def getZone(self, name):
        match name:
            case "PickupZone":
                return self.__pickup__
            case "EntryZone":
                return self.__entryzone__
            case "ExitZone":
                return self.__exitzone__
            #case "GroundZone":
            #    return self.__ground__
            case "BalanceZone":
                return self.__balance__
            case "ScoreRow":
                return self.__scoreRow__
            case "Community":
                return self.__community__

    def periodic(self):
        # Update Dashboards
        self.__pickup__.updateDashboard()
        self.__entryzone__.updateDashboard()
        self.__exitzone__.updateDashboard()
        #self.__ground__.updateDashboard()
        self.__balance__.updateDashboard()
        self.__scoreGrid__.updateDashboard()

class NavigationZone():
    def __init__(self, name:str):
        self.__name__:str = name
        self.__options__:list[dict] = []
        self.__selection__:int = -1
        self.__ntTbl__ = NetworkTableInstance.getDefault().getTable("Navigation")

    def addOption(self, name:str, blueValue:str, redValue:str):
        option = {
            "name": name,
            "blue": blueValue,
            "red" : redValue
        }
        self.__options__.append(option)
        if self.__selection__ == -1: self.setSelectedKey(0)

    def getOptions(self) -> list[dict]:
        return self.__options__
    
    def getSelectedValue(self) -> str:
        a = self.__options__[self.__selection__]
        if DriverStation.getAlliance() != DriverStation.Alliance.kRed:
            return self.__options__[self.__selection__].get("blue")
        else:
            return self.__options__[self.__selection__].get("red")
    
    def getSelectedName(self) -> str:
        return self.__options__[self.__selection__].get("name")

    def getSelectedIndex(self) -> str:
        return self.__selection__

    def setSelectedKey(self, index) -> None:
        minValue = 0
        maxValue = len( self.__options__ ) -1
        self.__selection__ = min( max( index, minValue ), maxValue )

    def move(self, direction:typing.Literal[-1,1]) -> None:
        newSelection = self.__selection__ + direction
        minValue = 0
        maxValue = len( self.__options__ ) - 1
        self.__selection__ = min( max( newSelection, minValue ), maxValue )

    def updateDashboard(self):
        name = self.getSelectedName()
        self.__ntTbl__.putString( f"{self.__name__}", f"{name}" )

class NavigationZoneGrid():
    def __init__(self, name:str, row:NavigationZone, column:NavigationZone):
        self.__name__ = name
        self.__row__:NavigationZone = row
        self.__col__:NavigationZone = column
        self.__ntTbl__:NetworkTable = NetworkTableInstance.getDefault().getTable("Navigation")

    def updateDashboard(self):
        row = self.__row__.getSelectedName()
        col = self.__col__.getSelectedName()
        self.__ntTbl__.putString( f"{self.__name__}", f"{row}-{col}" )

class NavigationToggleZone(InstantCommand):
    def __init__(self, navigation:Navigation):
        super().__init__()
        self.__nav__ = navigation

    def initialize(self) -> None:
        currentCmd = self.__nav__.getCurrentCommand()
        currentName = currentCmd.getName()
        currentCmd.cancel()
        match currentName:
            case "ScoreZone":
                NavigationUpdateZoneTimed( self.__nav__, "PickupZone" ).schedule()
            case "PickupZone":
                NavigationUpdateZoneTimed( self.__nav__, "EntryZone" ).schedule()
            case "ExitZone":
                NavigationUpdateZoneTimed( self.__nav__, "EntryZone" ).schedule()
            case "EntryZone":
                NavigationUpdateZoneTimed( self.__nav__, "BalanceZone" ).schedule()
            case "BalanceZone":
                #NavigationUpdateZoneTimed( self.__nav__, "GroundZone" ).schedule()
                pass
            case "GroundZone":
                pass #NavigationUpdateZoneValueTimed( self.nav, "ScoreZone" ).schedule()
            case _:
                pass
    
    def runsWhenDisabled(self) -> bool:
        return True

class NavigationUpdateZone(CommandBase):
    __pov__ = -1

    def __init__(self, navigation:Navigation, name:str):
        super().__init__()
        self.setName( f"{name}" )
        self.addRequirements( navigation )
        self.__name__ = name
        self.__nav__ = navigation

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        isRed = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        pov = self.__nav__.getPOV()
        name = self.__name__
        
        if pov != self.__pov__ and pov != -1:
            moveValue = 0

            if name == "ScoreZone":
                if pov == 0 or pov == 180:
                    name = "ScoreRow"
                elif pov == 90 or pov == 270:
                    name = "Community"
                else: return
            
            if pov == 0 or pov == 270:
                moveValue = -1
            elif pov == 90 or pov == 180:
                moveValue = 1
            else: return
            
            zone = self.__nav__.getZone( name )
            zone.move( moveValue )
        
        self.__pov__ = pov
        return super().execute()
    
    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return True

class NavigationUpdateZoneTimed(NavigationUpdateZone):
    __timer__ = Timer()

    def __init__(self, navigation:Navigation, name:str):
        super().__init__(navigation, name)

    def initialize(self) -> None:
        self.__timer__.reset()
        self.__timer__.start()

    def execute(self) -> None:
        super().execute()
        if self.__pov__ != -1:
            self.__timer__.reset()
        
    def end(self, interrupted: bool) -> None:
        self.__timer__.stop()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return self.__timer__.get() > 2.0