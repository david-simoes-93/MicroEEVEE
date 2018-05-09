# encoding: utf-8
# encoding: iso-8859-1
# encoding: win-1252
from astar import AStar
import sys
import math
from queue import *

WALL = 0
NO_WALL = 1
UNKNOWN = -1

CENTER_X = 8
CENTER_Y = 15

#intervalos em que garantidamente digo que tem ou nao parede
MIN_THRESHOLD = 50
MAX_THRESHOLD = 100

def between(t, v1, v2):
    return t >= v1 and t <= v2

class Maze(object):
    """docstring for Maze"""
    def __init__(self, h, w):
        self.w = w
        self.h = h
        self.limits = [[0,self.h],[0, self.w]]
        self.maze = [[Cell(x, y) for y in range(self.w)] for x in range(self.h)]
        self.ver = [["   "] * w + [' '] for _ in range(h)] + [[]]
        self.ver2 = [["   "] * w + [' '] for _ in range(h)] + [[]]
        self.hor = [["+  "] * w + ['+'] for _ in range(h + 1)]
        self.fullExplored = False


    def addWall(self,theta, _list_tmp, x1, y1 ): #OUT OF DATE

        _list = self.computeValues(theta, _list_tmp)
        #cell = self.computeCell(x1, y1)
        cell = [x1+8, -y1+15]

        if(_list == None or len(_list) < 4 ):
            print("ERROR!: list not complete")
            return
        else:
            #lambda x
            i = 0
            for val in _list:

                self.updateWall(i, cell, val)
                i+=1

    def computeValues(self, theta , l):
        if theta < 0 :
            theta += 2*math.pi
        deg = theta*360 / (2*math.pi) #counter clockwise

        if deg > 315 or deg < 45 : #front
            return l
        elif deg > 45 and deg < 135 :
            return [ l[3] ,l[0] ,l[1] ,l[2] ]
        elif deg > 135 and deg < 225 :
            return [ l[1] , l[2] , l[3] , l[0] ]
        else :
            return [ l[2] , l[3] , l[0] , l[1] ]

    def computeCell(self,x1, y1): #WRONG
        #já o x e y em relação à grid
        x2 = int(round(x1/2))
        y2 = int(round(y1/2))

        cell_x = x2 + CENTER_X
        cell_y = y2 + CENTER_Y

        return [cell_x, cell_y]

    def updateLimits(self, x, y):
        diff = x - CENTER_X

        if x >= CENTER_X :
            self.limits[0][0] = max (diff  , self.limits[0][0] )
        else :
            self.limits[0][1] =  min (CENTER_X - diff , self.limits[0][1] )
        if(self.limits[0][1] - self.limits[0][0] < CENTER_X ):
            print('ERROR ! : grid out of bounds (x)')

        diff = y - CENTER_Y

        if y >= CENTER_Y :
            self.limits[1][0] = max (diff  , self.limits[1][0] )
        else :
            self.limits[1][1] =  min (CENTER_Y - diff  , self.limits[1][1] )
        if(self.limits[1][1] - self.limits[1][0] < CENTER_Y ):
            print('ERROR ! : grid out of bounds (y)')


    def updateWall(self,i, cell, val):
        x = cell[0]
        y = cell[1]
        f_val = 0
        if i == 0 :
            if val > 0 : # WALL
                if 0 <= y+1 < self.w :
                    f_val = max(self.maze[x][y].walls[0] , self.maze[x][y+1].walls[2] )
                    self.maze[x][y].walls[0] = f_val + val
                    self.maze[x][y+1].walls[2] = f_val + val
                    self.maze[x][y+1].update_vis()
                else:
                    f_val = self.maze[x][y].walls[0]
                    self.maze[x][y].walls[0] = f_val + val
            else: #NO_WALL

                if 0 <= y+1 < self.w :
                    f_val = min(self.maze[x][y].walls[0] , self.maze[x][y+1].walls[2] )
                    self.maze[x][y].walls[0] = f_val + val
                    self.maze[x][y+1].walls[2] = f_val + val
                    self.maze[x][y+1].update_vis()
                else:
                    f_val = self.maze[x][y].walls[0]
                    self.maze[x][y].walls[0] = f_val + val
        elif i == 1 :
            if val > 0 : # WALL
                if 0 <= x+1 < self.w :
                    f_val = max(self.maze[x][y].walls[1] , self.maze[x+1][y].walls[3] )
                    self.maze[x][y].walls[1] = f_val + val
                    self.maze[x+1][y].walls[3] = f_val + val
                    self.maze[x+1][y].update_vis()
                else:
                    f_val = self.maze[x][y].walls[1]
                    self.maze[x][y].walls[1] = f_val + val
            else: #NO_WALL
                if 0 <= x+1 < self.h :
                    f_val = min(self.maze[x][y].walls[1] , self.maze[x+1][y].walls[3] )
                    self.maze[x][y].walls[1] = f_val + val
                    self.maze[x+1][y].walls[3] = f_val + val
                    self.maze[x+1][y].update_vis()
                else:
                    f_val = self.maze[x][y].walls[1]
                    self.maze[x][y].walls[1] = f_val + val
        elif i == 2 :
            if val > 0 :
                if 0 <= y-1 < self.w :
                    f_val = max(self.maze[x][y].walls[2] , self.maze[x][y-1].walls[0] )
                    self.maze[x][y].walls[2] = f_val + val
                    self.maze[x][y-1].walls[0] = f_val + val
                    self.maze[x][y-1].update_vis()
                else:
                    f_val = self.maze[x][y].walls[2]
                    self.maze[x][y].walls[2] = f_val + val
            else: #NO_WALL
                if 0 <= y-1 < self.w :
                    f_val = min(self.maze[x][y].walls[2] , self.maze[x][y-1].walls[0] )
                    self.maze[x][y].walls[2] = f_val + val
                    self.maze[x][y-1].walls[0] = f_val + val
                    self.maze[x][y-1].update_vis()
                else:
                    f_val = self.maze[x][y].walls[2]
                    self.maze[x][y].walls[2] = f_val + val
        elif i == 3 :
            if val > 0 : # WALL
                if 0 <= x-1 < self.w :
                    f_val = max(self.maze[x][y].walls[3] , self.maze[x-1][y].walls[1] )
                    self.maze[x][y].walls[3] = f_val + val
                    self.maze[x-1][y].walls[1] = f_val + val
                    self.maze[x-1][y].update_vis()
                else:
                    f_val = self.maze[x][y].walls[3]
                    self.maze[x][y].walls[3] = f_val + val
            else: #NO_WALL
                if 0 <= x-1 < self.h :
                    f_val = min(self.maze[x][y].walls[3] , self.maze[x+1][y].walls[1] )
                    self.maze[x][y].walls[3] = f_val + val
                    self.maze[x-1][y].walls[1] = f_val + val
                    self.maze[x-1][y].update_vis()
                else:
                    f_val = self.maze[x][y].walls[3]
                    self.maze[x][y].walls[3] = f_val + val
        else:
            print("i %d (%d, %d)" %(i, x, y))
            print("ERROR")

        self.maze[x][y].update_vis()

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

    def drawMaze_visited(self):
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
                    if self.maze[i][j].vis == 1:
                        self.ver2[i][j] = "| *"
                    else:
                        self.ver2[i][j] = "|  "
                elif j == self.w and self.maze[i][j-1].hasWall(0) == WALL :
                    self.ver2[i][j] = "|"
                elif j < self.w and self.maze[i][j].hasWall(2) == NO_WALL and self.maze[i][j].vis == 1 :
                    self.ver2[i][j] = " * "
    

        result  = ''
        for (a, b) in zip(self.hor, self.ver2):
            result = result + (''.join(a + ['\n'] + b)) + '\n'


        return result.strip()


    def findNext(self, node , theta):
        """
        cur_pos em relação ao maze
        node for the map

        """

        q = Queue()
        tmp = []

        if not self.maze[ node[0]][node[1]].vis :
            return [node]
        else:
            q.put( [node] )
            tmp+=[node]
            while True:

                while not q.empty() :
                    node = q.get() # [(1,2)] or [(2,2), (1,2)]  or....
                    #print ("node now:",node)
                    #neighbors = self.neighbors(node[0], theta)
                    neighbors = self.neighbor_North(node[0], theta) + self.neighbor_East(node[0], theta) + self.neighbor_West(node[0], theta) + self.neighbor_South(node[0], theta)
                    for x in neighbors:
                        if not self.maze[ x[0]][x[1]].vis :
                            return node[::-1] + [x]
                        else:
                            if x not in tmp:
                                q.put([x]+node)
                                tmp += [x]

                if q.empty():
                    fullExplored = True
                    break
        return []

    """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
        nodes that can be reached (=any adjacent coordinate that is not a wall)
    """
    def neighbor_North(self, node, theta):
        x, y = node
        if theta < 0 :
            theta += 2*math.pi
        deg = theta*360 / (2*math.pi) #counter clockwise


        if deg > 315 or deg < 45 : #front
            nx , ny = x, y + 1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(0) == NO_WALL):
                return [(nx,ny)]
        elif deg > 45 and deg < 135 :
            nx , ny = x - 1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(3) == NO_WALL):
                return [(nx,ny)]
        elif deg > 135 and deg < 225 :
            nx , ny = x, y - 1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(2) == NO_WALL):
                return [(nx,ny)]
        else :
            nx , ny = x + 1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(1) == NO_WALL):
                return [(nx,ny)]
        return []

    def neighbor_East(self, node, theta):
        x, y = node
        if theta < 0 :
            theta += 2*math.pi
        deg = theta*360 / (2*math.pi) #counter clockwise


        if deg > 315 or deg < 45 : #right
            nx , ny = x +1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(1) == NO_WALL):
                return [(nx,ny)]
        elif deg > 45 and deg < 135 :
            nx , ny = x , y +1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(0) == NO_WALL):
                return [(nx,ny)]
        elif deg > 135 and deg < 225 :
            nx , ny = x-1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(3) == NO_WALL):
                return [(nx,ny)]
        else :
            nx , ny = x , y-1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(2) == NO_WALL):
                return [(nx,ny)]
        return []

    def neighbor_West(self, node, theta):
        x, y = node
        if theta < 0 :
            theta += 2*math.pi
        deg = theta*360 / (2*math.pi) #counter clockwise


        if deg > 315 or deg < 45 : #front
            nx , ny = x-1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(3) == NO_WALL):
                return [(nx,ny)]
        elif deg > 45 and deg < 135 :
            nx , ny = x , y-1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(2) == NO_WALL):
                return [(nx,ny)]
        elif deg > 135 and deg < 225 :
            nx , ny = x +1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(1) == NO_WALL):
                return [(nx,ny)]
        else :
            nx , ny = x , y +1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(0) == NO_WALL):
                return [(nx,ny)]
        return []

    def neighbor_South(self, node, theta):
        x, y = node
        if theta < 0 :
            theta += 2*math.pi
        deg = theta*360 / (2*math.pi) #counter clockwise


        if deg > 315 or deg < 45 : #front
            nx , ny = x, y - 1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(2) == NO_WALL):
                return [(nx,ny)]
        elif deg > 45 and deg < 135 :
            nx , ny = x + 1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(1) == NO_WALL):
                return [(nx,ny)]
        elif deg > 135 and deg < 225 :
            nx , ny = x, y + 1
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(0) == NO_WALL):
                return [(nx,ny)]
        else :
            nx , ny = x - 1, y
            if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(3) == NO_WALL):
                return [(nx,ny)]
        return []

    def neighbors(self, node, theta):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node

        #para theta igual a zero

        l = [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
        result = []

        nx , ny = l[1]
        if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(0) == NO_WALL):
            result+=[(nx,ny)]

        nx , ny = l[3]
        if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(1) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[2]
        if (0  <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(3) == NO_WALL):
            result+=[(nx,ny)]

        nx , ny = l[0]
        if ( 0 <= nx < self.h and 0 <= ny < self.w and self.maze[x][y].hasWall(2) == NO_WALL):
            result+=[(nx,ny)]

        # print(node,result)
        return result


class Cell(object):
    """docstring for Cell

    YAM strategy
    -> peso de confiança (mais perto , + valor) (menos perto , - valor) [100 - 1]
    -> [-100, 0, 100] : (0 : unknown) ( < -40 : no wall ) (> 40 : wall )

    """
    def __init__(self, x, y):
        # 0 - front, 1-right, 2-back, 3-left
        self.walls = [0,0,0,0]
        # 0 : not visited
        self.vis = 0
        self.x = x
        self.y = y

    def add (self, i, val):
        self.walls[i] += val
        if math.abs(self.walls[i]) > MAX_THRESHOLD :
            if self.walls[i] < -MAX_THRESHOLD :
                self.walls[i] = -MAX_THRESHOLD
            else:
                self.walls[i] = MAX_THRESHOLD
        self.update_vis()

    def update_vis(self):
        b = False
        for i in range(0, 4):
            if self.hasWall(i) == UNKNOWN :
                b = True
                self.vis = 0
                break
        if b== False :
            self.vis = 1


    def hasWall(self, index):
        """
        for now [-100 -50] : not a wall
                [50 100] : wall
                ]-50 50[ : unknown
        """
        if self.walls[index] >= MIN_THRESHOLD :
            return WALL
        elif self.walls[index] <= -MIN_THRESHOLD:
            return NO_WALL
        return UNKNOWN

    def __str__(self):
        d = {WALL : 'W' , NO_WALL: 'N' , UNKNOWN : 'U'}
        r = ''
        r +="--" + d[self.hasWall(3)] + "--"+'\n'
        r +=d[self.hasWall(2)] + str(self.x) + ',' + str(self.y) + d[self.hasWall(0)] + '\n'
        r += "--" + d[self.hasWall(1)] + "--"+ '\n'
        return r

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



if __name__ == "__main__": #OUT OF DATE
    #7,14
    mp = Maze(15, 29) #com margens de erro

    d = {(0,0) : [100, -100, 100, 100] , (0,1) : [-100, -100, 100, 100] , (0,2) : [-100, -100, -100, 100] , (0,3) : [100, -100, -100, 100] , (1,0) : [100, -100, 100, -100], (1,1) :[-100, 100, -100, -100] , (1,2) : [-100, -100, -100, -100], (1,3) : [1000, -100, -100, -100] , (2,0) : [-100, 100, 100, -100]  , (2,1) : [-100, 100, -100, 100] , (2,2) : [100, 100, -100, -100] , (2,3) :[100, 100, 100, -100]}

    t = [(8,15)]
    (x1, y1) = t[-1]
    print("clear")
    mp.addWall(0, [100, -100, 100, 100] , 0,0)
    print ("visited", mp.maze[8][15].vis)
    inter = 0
    while(True):
        t = mp.findNext((x1,y1),math.pi)
        inter += len(t)
        if(t == []):
            break
        (x1, y1) = t[-1]

        l = d[(x1-8, y1-15)]
        print("list",x1, y1, l)


        mp.addWall(0, l , x1-8, y1-15 )
        #print ("visited",mp.maze[x1][y1].vis ,"\n walls" , mp.maze[x1][y1].walls)
    print("interations: ", inter)

    # mp.addWall(0, [1, 0, 1, 1] , 0,0)
    # mp.addWall(0, [0, 0, 1, 1] , 0,2)
    # mp.addWall(0, [0, 0, 0, 1] , 0,4)
    # mp.addWall(0, [1, 0, 0, 1] , 0,6)

    # mp.addWall(0, [-1, 0, 1, 0] , 2,0)
    # #mp.addWall(0, [0, 1, 0, 0] , 1,1)
    # mp.addWall(0, [0, 0, 0, 0] , 2,4)
    # mp.addWall(0, [1, 0, 0, 0] , 2,6)
    # mp.addWall(0, [0, 1, 1, 0] , 4,0)

    # mp.addWall(0, [0, 1, 0, 1] , 4,2)
    # mp.addWall(0, [1, 1, 0, 0] , 4,4)
    # mp.addWall(0, [1, 1, 1, 0] , 4,6)



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
