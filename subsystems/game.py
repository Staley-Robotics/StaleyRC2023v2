import typing
from commands2 import *

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
