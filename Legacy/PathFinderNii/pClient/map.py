
from astar import AStar
import sys
import math

WALL = 0
NO_WALL = 1
UNKNOWN = -1

def between(t, v1, v2):
    return t >= v1 and t <= v2

class Maze(object):
    """docstring for Maze"""
    def __init__(self, h, w):
        self.w = w
        self.h = h
        self.limits = [[0,self.h],[0, self.w]]
        self.maze = [[Cell() for y in range(self.w)] for x in range(self.h)]
        self.ver = [["   "] * w + [' '] for _ in range(h)] + [[]]
        self.hor = [["+  "] * w + ['+'] for _ in range(h + 1)]

    def addWall(self,theta, _list_tmp, x1, y1 ):
        
        _list = self.computeValues(theta, _list_tmp)
        cell = self.computeCell(x1, y1)
        #cell = [x1, y1]
        print (cell)
        if(_list == None or len(_list) < 4 ):
            print("ERROR!: list not complete")
            return 
        else:
            #lambda x
            i = 0
            for val in _list:
                if(val != -1):
                    self.updateWall(i, cell, val)
                i+=1

    def computeValues(self, t , l):
        # [-30 , 30] front
        if between(t, -0.52, 0.52):
            return l
        #[ 60, 120]
        elif between(t, 1.04 , 2.09):
            return [ l[1] , l[2] , l[3] , l[0] ]
        elif between(t, 2.61, 3.14)  or between(t, -3.14, -2.61):
            return [ l[2] , l[3] , l[0] , l[1] ]
        elif between(t, -2.09, -1.04):
            return [ l[3] ,l[0] ,l[1] ,l[2] ]
        else: 
            return None

    def computeCell(self,x1, y1):
        x2 = int(x1/2)
        y2 = int(y1/2)
        
        cell_x = x2 + 8
        cell_y = y2 + 15
        
        self.updateLimits(cell_x, cell_y)

        return [cell_x, cell_y]

    def updateLimits(self, x, y):
        diff = x - 7

        if x >= 7 :
            self.limits[0][0] = max (diff  , self.limits[0][0] )
        else :
            self.limits[0][1] =  min (7 - diff , self.limits[0][1] )
        if(self.limits[0][1] - self.limits[0][0] < 7 ):
            print('ERROR ! : grid out of bounds (x)')
        
        diff = y - 14

        if y >= 14 :
            self.limits[1][0] = max (diff  , self.limits[1][0] )
        else :
            self.limits[1][1] =  min (14 - diff  , self.limits[1][1] )
        if(self.limits[1][1] - self.limits[1][0] < 14 ):
            print('ERROR ! : grid out of bounds (y)')
        

    def updateWall(self,i, cell, val):
        x = cell[0] 
        y = cell[1]
        #front wall
        if(i== 0 ): #DONE
            if(y < self.w-1):
                f_val = max(val, self.maze[x][y].walls[0] , self.maze[x][y+1].walls[2] )
                self.maze[x][y].walls[0] = f_val
                self.maze[x][y+1].walls[2] = f_val
            elif(y == self.w-1):
                f_val = max(val, self.maze[x][y].walls[0]  )
                self.maze[x][y].walls[0] = f_val
            #right wall
        elif(i==1 ) : #DONE
            if(x < self.h-1):
                f_val = max(val, self.maze[x][y].walls[1] , self.maze[x+1][y].walls[3] )
                self.maze[x][y].walls[1] = f_val
                self.maze[x+1][y].walls[3] = f_val
            elif(x == self.h-1):
                f_val = max(val, self.maze[x][y].walls[1]  )
                self.maze[x][y].walls[1] = f_val
        
        elif(i == 2 ): #DONE
            if(y > 0):
                f_val = max(val, self.maze[x][y].walls[2] , self.maze[x][y-1].walls[0] )
                self.maze[x][y].walls[2] = f_val
                self.maze[x][y-1].walls[0] = f_val
            elif(y== 0):
                f_val = max(val, self.maze[x][y].walls[2] )
                self.maze[x][y].walls[2] = f_val
                
        elif(i==3 ):
            if(x > 0):
                f_val = max(val, self.maze[x][y].walls[3] , self.maze[x-1][y].walls[1] )
                self.maze[x][y].walls[3] = f_val
                self.maze[x-1][y].walls[1] = f_val
            elif(x== 0):
                f_val = max(val, self.maze[x][y].walls[3] )
                self.maze[x][y].walls[3] = f_val
        else:
            print("i %d (%d, %d)" %(i, x, y))
            print("ERROR")
        
        return

    def drawMaze(self, set1=[], set2=[], c='#', c2='*' ):
        """returns an ascii maze, drawing eventually one (or 2) sets of positions.
            useful to draw the solution found by the astar algorithm
        """
        #hor
        for i in range(0, self.h +1 ):

            for j in range(0, self.w):
                if i < self.h and self.maze[i][j].hasWall(3) == WALL :
                    self.hor[i][j] = "+--"
                elif i == self.h and self.maze[i-1][j].hasWall(1) == WALL  :
                    self.hor[i][j] = "+--"
                


        #ver
        for i in range(0, self.h ):

            for j in range(0, self.w +1):
                if j < self.w and self.maze[i][j].hasWall(2) == WALL :
                    if (i,j) in set1:
                        self.ver[i][j] = "| #"
                    else:
                        self.ver[i][j] = "|  "
                elif j == self.w and self.maze[i][j-1].hasWall(0) == WALL :
                    self.ver[i][j] = "|"
                elif j < self.w and self.maze[i][j].hasWall(2) == NO_WALL and (i,j)in set1 :
                    self.ver[i][j] = "###"
        #[(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (1, 1), (0, 1)]
        for i in range(1, len(set1)):
            (x1,y1) = set1[i-1]
            (x2, y2) = set1[i]
            

            if y2==y1 :
            
                if x2 == max(x2, x1) :
                    self.hor[x2][y2] = "+ #"
                else:
                    self.hor[x1][y1] = "+ #"

        result  = ''
        for (a, b) in zip(self.hor, self.ver):
            result = result + (''.join(a + ['\n'] + b)) + '\n'
        

        return result.strip()


        
class Cell(object):
    """docstring for Cell

    YAM strategy
    -> peso de confiança (mais perto , + valor) (menos perto , - valor) [100 - 1]
    -> [-100, 0, 100] : (0 : unknown) ( < -40 : no wall ) (> 40 : wall )

    """
    def __init__(self):
        # 0 - front, 1-right, 2-back, 3-left
        self.walls = [-1,-1,-1,-1]
        # 0 : not visited
        self.vis = 0

    def hasWall(self, index):
        """
        for now 0 : not a wall
                1 : wall
                -1 : unknown 
        """
        if self.walls[index] == 1 :
            return WALL
        elif self.walls[index] == 0:
            return NO_WALL
        return UNKNOWN 


class MazeSolver(AStar):

    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, maze):
        self.lines = maze.maze
        # self.lines = maze.strip().split('\n')
        
        self.width = len(self.lines[0])
        self.height = len(self.lines)
        self.minX = 0
        self.minY = 0


    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2

        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        l = [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
        result = []

        nx , ny = l[0]
        if ( self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(2) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[1]
        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(0) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[2]
        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(3) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[3]
        
        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(1) == NO_WALL):
            result+=[(nx,ny)]

        # print(node,result)
        return result
        

#7,14
mp = Maze(15, 29) #com margens de erro


mp.addWall(0, [1, 0, 1, 1] , 0,0)
mp.addWall(0, [0, 0, 1, 1] , 0,2)
mp.addWall(0, [0, 0, 0, 1] , 0,4)
mp.addWall(0, [1, 0, 0, 1] , 0,6)

mp.addWall(0, [-1, 0, 1, 0] , 2,0)
#mp.addWall(0, [0, 1, 0, 0] , 1,1)
mp.addWall(0, [0, 0, 0, 0] , 2,4)
mp.addWall(0, [1, 0, 0, 0] , 2,6)
mp.addWall(0, [0, 1, 1, 0] , 4,0)

mp.addWall(0, [0, 1, 0, 1] , 4,2)
mp.addWall(0, [1, 1, 0, 0] , 4,4)
mp.addWall(0, [1, 1, 1, 0] , 4,6)



print (mp.drawMaze())


start = (0+8,0+15)
goal = (2+8,3+15)

# let's solve it
foundPath = list(MazeSolver(mp).astar(start, goal))

print(foundPath)
# print the solution
print(mp.drawMaze( list(foundPath)))
print ('limits x: ',mp.limits[0][0] , mp.limits[0][1] )
print('limits y: ', mp.limits[1][0] , mp.limits[1][1])