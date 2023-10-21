import typing
from commands2 import *

class ChargedUp:
    class Rows:
        ArenaNear = 0
        DropoffTop = 0.385
        DropoffMiddle = 0.817
        DropoffBottom = 1.175
        DropoffEdge = 1.425
        DropoffRobotEdge = 1.831
        DropoffRobotNormal = 2.034
        DropoffRobotMiddle = 2.175
        ChargeNear = 2.925
        ChargeCenter = 3.9075
        ChargeFar = 4.89
        SafeExit = 5.296
        ObjectPlace = 7.075
        ArenaMiddle = 8.2615
        PickupEntry = 12.75
        PickupRobot = 14.225
        PickupRobotNormal = 15.576
        PickupRobotEdge = 15.779
        PickupEdge = 16.185
        ArenaFar = 16.523

    class Columns:
        class Blue:
            ArenaLeft = 8.013
            PickupSide = 7.607
            PickupLeft = 7.35
            PickupEntry = 6.775
            PickupRight = 6.2
            PickupDropoffEdge = 5.475
            Dropoff1 = 4.98
            SafeExitLeft = 4.725
            ObjectPlace1 = 4.575
            Dropoff2 = 4.425
            ChargeEdgeLeft = 3.975
            Dropoff3 = 3.865
            ChargeBalanceLeft = 3.525
            ObjectPlace2 = 3.365
            Dropoff4 = 3.305
            ChargeBalanceMiddle = 2.75
            Dropoff5 = 2.735
            Dropoff6 = 2.18
            ObjectPlace3 = 2.135
            ChargeBalanceRight = 1.95
            Dropoff7 = 1.625
            ChargeEdgeRight = 1.525
            Dropoff8 = 1.075
            ObjectPlace4 = 0.925
            SafeExitRight = 0.7625
            Dropoff9 = 0.5
            ArenaRight = 0

    class Blue:
        a = 1
    class Red:
        a = 1

class Game(SubsystemBase):
    _pickup = 1
    _dropRow = 3
    _dropColumn = 6

    def __init__(self):
        super().__init__()
        pass

    ### Pickup Functions
    def getPickup(self) -> int:
        return self._pickup
    
    def setPickup(self, location:int) -> None:
        self._pickup = location

    ### Drop Functions
    def getDropoff(self) -> list([int,int]):
        return [self._dropRow,self._dropColumn]

    def setDropoff(self, row:int, column:int) -> None:
        self.setDropoffRow(row)
        self.setDropoffColumn(column)

    ### Drop Row Functions
    def getDropoffRow(self) -> int:
        return self._dropRow

    def setDropoffRow(self, row:int) -> None:
        row = max( min( row, 3 ), 1 )
        self._dropRow = row

    ### Drop Column Functions
    def getDropoffColumn(self) -> int:
        return self._dropColumn

    def setDropoffColumn(self, column:int) -> None:
        column = max( min( column, 9 ), 1 )
        self._dropColumn = column


class SelectPickup(CommandBase):
    pass

class SelectDropoff(CommandBase):
    lastPov = -1

    def __init__( self, game:Game, input:typing.Callable[[],int] ):
        super().__init__()
        self.addRequirements( game )
        self.game = game
        self.input = input

    def initialize(self):
        pass
        
    def execute(self):
        pov = self.input()
        
        if pov == self.lastPov: return
        self.lastPov = pov        
        
        dropOff = self.game.getDropoff()
        print( "Row: ", dropOff[0] )
        print( "Col: ", dropOff[1] )
        match pov:
            # Left
            case 90: #,45, 135:
                self.game.setDropoffColumn( dropOff[1]+1 )
            # Right
            case 270: #,225, 315:
                self.game.setDropoffColumn( dropOff[1]-1 )
            # Up
            case 0: #, 315, 45:
                self.game.setDropoffRow( dropOff[0]+1 )
            # Down
            case 180: #,135, 225:
                self.game.setDropoffRow( dropOff[0]-1 )
            case _:
                return

    def end(self, interrupted:bool) -> None:
        pass
    
    def isFinished(self) -> bool:
        return False
    
    def runsWhenDisabled(self) -> bool:
        return True
