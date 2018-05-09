from math import *
import numpy as np
from mapgrid import * 

MIN_DIST = 1
MAX_DIST = 4

TRUST_MIN = 1
TRUST_MAX = 0

CONSTANT = 50

def divide(val):
    if val < 0 :
        return -int(round(abs(val) /2))
    return int(round(val/2))


def translate(value):
    # Figure out how 'wide' each range is
    leftSpan = MAX_DIST - MIN_DIST
    rightSpan = TRUST_MAX - TRUST_MIN

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - MIN_DIST) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return TRUST_MIN + (valueScaled * rightSpan)

def find_Near_Odd(num):
    if int(num) % 2 == 0:
        #par  -0.25000000000000006
        p = int(num)
        k1 = p - 1
        k2 = p + 1
        """
        quando sao iguais, nao sei o que fazer, mas em principio nunca calha iguais
        """
        if (abs(k1 - num) > abs(k2 - num)):
            return k2
        elif (abs(k1 - num) <= abs(k2 - num)):
            return k1
        else:
            print("ERROR: cannot find nearest odd number!")
            return None
    else:
        return int(num)

def found_odds(v1, v2):
    init = min(v1, v2)
    end = max(v1, v2)
    i = int(init)
    l=[]
    while True:
        if(i >= init and i <= end ):
            if i% 2 != 0 : #odd
                l+=[i]
        elif(i > end):
            if v1 > v2 :
                return l
            else:
                return l[::-1]
            
        i+=1



class Sensors(object):
    
    """docstring for Sensors"""
    def __init__(self ):
        """ 
            [-30 0 30]
        """
        
        self.median_r = Buffer(3)
        self.median_l = Buffer(3)
        self.median_f = Buffer(3)
        self.median_b = Buffer(3)
    
    def add(self, l, r, f, b):
        """
            add new value from Sensors
        """
        self.median_f.add(f)
        self.median_l.add(l)
        self.median_r.add(r)
        self.median_b.add(b)

    def update_map(self, map_, rx, ry, theta ):
        if theta < 0:
            t = theta+(2*pi)
        else:
            t = theta
        
        print("front")
        self.calc_walls(map_, rx, ry, t + 0, self.median_f.get())
        print("left") ##VER ISTO
        self.calc_walls(map_, rx, ry, t + radians(60), self.median_l.get())
        print("bavk")
        self.calc_walls(map_, rx, ry, t + radians(-180), self.median_b.get())
        print("right")
        self.calc_walls(map_, rx, ry, t + radians(-60), self.median_r.get())

    def update_map_v3(self, map_, rx, ry, theta, f, l, b, r ):
        if theta < 0:
            t = theta+(2*pi)
        else:
            t = theta
        
        # g_y = divide(rx) + 15
        # g_x = abs(divide(ry) - 8 )
        # c = map_.maze[g_x][g_y]
        # if c.vis == 1 :
        #     print"VISITED"
        #     print(map_.maze[g_x][g_y])
        #     return
        #print("front")
        self.calc_walls_v3(map_, rx, ry, t, f, 0 )
        #print("left") ##VER ISTO
        self.calc_walls_v3(map_, rx, ry, t, l, 3)
        #print("bavk")
        self.calc_walls_v3(map_, rx, ry, t, b , 2)
        #print("right")
        self.calc_walls_v3(map_, rx, ry, t, r, 1)

        #print cell
        g_y = divide(rx) + 15
        g_x = abs(divide(ry) - 8 )
        print(map_.maze[g_x][g_y])
        return (g_x, g_y)


    def update_map_v2(self, map_, rx, ry, theta, f, l, b, r ):
        if theta < 0:
            t = theta+(2*pi)
        else:
            t = theta
        
        # g_y = divide(rx) + 15
        # g_x = abs(divide(ry) - 8 )
        # c = map_.maze[g_x][g_y]
        # if c.vis == 1 :
        #     print"VISITED"
        #     print(map_.maze[g_x][g_y])
        #     return
        #print("front")
        self.calc_walls_v2(map_, rx, ry, t, f, 0 )
        #print("left") ##VER ISTO
        self.calc_walls_v2(map_, rx, ry, t, l, 3)
        #print("bavk")
        self.calc_walls_v2(map_, rx, ry, t, b , 2)
        #print("right")
        self.calc_walls_v2(map_, rx, ry, t, r, 1)

        #print cell
        g_y = divide(rx) + 15
        g_x = abs(divide(ry) - 8 )
        print(map_.maze[g_x][g_y])
        return (g_x, g_y)

    def calc_walls_v2(self, map_, rx, ry, theta, dist, index):
        """
            theta [0, 2pi]
        """
        #cur_cell = ...
        g_y = divide(rx) + 15
        g_x = abs(divide(ry) - 8 )

        #find what wall index regarding theta of robot
        deg = theta*360 / (2*pi) #counter clockwise
        wall_i = -1
        if deg > 315 or deg < 45 : #front
            wall_i = index
        elif deg > 45 and deg < 135 : #left
            l = [3 ,0 ,1 ,2]
            wall_i = l[index] 
        elif deg > 135 and deg < 225 : #back
            l = [1 ,2 ,3 ,0]
            wall_i = l[index] 
        else : #right
            l = [2, 3 ,0 ,1 ]
            wall_i = l[index] 


        if dist <= 1 :
            #mark as WALL
            map_.updateWall(wall_i , (g_x, g_y) , 50)
        elif dist > 1.5 :
            #mark as no_wall
            map_.updateWall(wall_i , (g_x, g_y) , -50)
        #put the rest TODO
            

    def calc_walls_v3(self, map_, rx, ry, theta, dist, index) :
        """
            theta [0, 2pi]
        """
        #cur_cell = ...
        g_y = divide(rx) +15
        g_x = abs(divide(ry) - 8 ) 

        #find what wall index regarding theta of robot
        deg = theta*360 / (2*pi) #counter clockwise
        wall_i = -1
        if deg > 315 or deg < 45 : #front
            wall_i = index
        elif deg > 45 and deg < 135 : #left
            l = [3 ,0 ,1 ,2]
            wall_i = l[index] 
        elif deg > 135 and deg < 225 : #back
            l = [2, 3 ,0 ,1 ]
            wall_i = l[index] 
        else : #right
            l = [1 ,2 ,3 ,0] ##ERRO
            wall_i = l[index] 


        if dist < 0.5  :
            #mark as WALL
            map_.updateWall(wall_i , (g_x, g_y) , 100)
        elif dist > 2.0 :
            #mark as no_wall
            map_.updateWall(wall_i , (g_x, g_y) , -100)
        else:
            #calculate position of the wall being mapped
            if wall_i == 0: 

                pos = (g_y-15) * 2 + 1 

                d2 = hypot(rx - pos, ry - ry) - 0.5
                print ("i: ", wall_i, " |dist ", dist , " pos ", pos , " d2 ", d2+0.4)
                if dist > d2 + 0.4 :
                    #mark as no_wall
                    map_.updateWall(wall_i , (g_x, g_y) , -50)
                    
                else:
                    #mark as WALL
                    map_.updateWall(wall_i , (g_x, g_y) , 50)
            elif wall_i == 2: 
                pos = (g_y-15) * 2 - 1 
                d2 = hypot(rx - pos, ry - ry) - 0.5
                print ("i: ", wall_i, " |dist ", dist , " pos ", pos , " d2 ", d2+0.4)
                if dist > d2 + 0.4 :
                    #mark as no_wall
                    map_.updateWall(wall_i , (g_x, g_y) , -50)
                else:
                    #mark as WALL
                    map_.updateWall(wall_i , (g_x, g_y) , 50)
            elif wall_i == 1: 
                beta = -1
                pos = (g_x-8) * 2 
                pos = -pos +  beta
                d2 = hypot(rx - rx, ry - pos) - 0.5
                print ("i: ", wall_i, " |dist ", dist , " pos ", pos , " d2 ", d2+0.4)
                if dist > d2 + 0.4 :
                    #mark as no_wall
                    map_.updateWall(wall_i , (g_x, g_y) , -50)
                else:
                    #mark as WALL
                    map_.updateWall(wall_i , (g_x, g_y) , 50)
            elif wall_i == 3: 
                beta = +1
                pos = (g_x-8) * 2 
                pos = -pos +  beta

                d2 = hypot(rx - rx, ry - pos) - 0.5
                print ("i: ", wall_i, " |dist ", dist , " pos ", pos , " d2 ", d2+0.4)
                if dist > d2 + 0.4 :
                    #mark as no_wall
                    map_.updateWall(wall_i , (g_x, g_y) , -50)
                else:
                    #mark as WALL
                    map_.updateWall(wall_i , (g_x, g_y) , 50)
        #put the rest TODO
            

        
    def calc_walls(self, map_, rx, ry, theta, di):
        #front
        dist = di
        ix = []
        iy = []
        s_x = [0,0,0]
        s_y = [0,0,0]
        if dist <= 4 : #encontrou parede
            trust = translate( dist )

            

            #where is the wall
            s_x[0] = cos( radians(-30) + theta) * dist +  cos(theta)*0.5 + rx
            s_x[1] = cos( radians(0) + theta) * dist +  cos(theta)*0.5 + rx
            s_x[2] = cos( radians(30) + theta) * dist +  cos(theta)*0.5 + rx

            s_y[0] = sin( radians(-30) + theta) * dist +  sin(theta)*0.5 + ry
            s_y[1] = sin( radians(0) + theta) * dist +  sin(theta)*0.5 + ry
            s_y[2] = sin( radians(30) + theta) * dist +  sin(theta)*0.5 + ry


            for i in range(0, 3) :
                ix += [find_Near_Odd(s_x[i])]
                iy += [find_Near_Odd(s_y[i])]
                

                k1 = abs( s_x[i] - ix[i])
                k2 = abs( s_y[i] - iy[i])
                print("ix", ix[i], "iy", iy[i] , s_x[i], s_y[i] , k1, k2 )
                #podemos impor ainda ao k que for menor que seja inferior a uma valor de x robot units
                if ( k1 < k2) :
                    #parede index 0 - frente para a celula (g_x, g_y)
                    g_y = divide(ix[i]) + 15
                    g_x = abs(divide(iy[i]) - 8 )
                    print ("g_y (" , int(ix[i]/2)  , ")", g_y)
                    if ix[i] < 0 :
                        wall_index = 2
                    else:
                        wall_index = 0
                    map_.updateWall(wall_index, (g_x, g_y) , CONSTANT/3* trust)
                    print(i, ": wall ", wall_index, " cell ",(g_x, g_y) , CONSTANT/3* trust)
                elif ( k1 > k2 ) :
                    #parede index 0 - frente para a celula (g_x, g_y)
                    g_y = divide(ix[i]) + 15
                    g_x = abs(divide(iy[i]) - 8 )

                    if iy[i] < 0 :
                        wall_index = 1
                    else:
                        wall_index = 3
                    map_.updateWall(wall_index, (g_x, g_y) , CONSTANT/3* trust)
                    print(i, ": wall ", wall_index, " cell ",(g_x, g_y) , CONSTANT/3* trust)

                #else --> error (meio da celula)

        if(dist > 1) : #nao encontrou parede
            
            if(ix == [] or iy ==[]): #nao teve paredes
                dist = 4
                #calcular para o max de 4
                s_x[0] = cos( radians(-30) + theta) * dist +  cos(theta)*0.5 + rx
                s_x[1] = cos( radians(0) + theta) * dist +  cos(theta)*0.5 + rx
                s_x[2] = cos( radians(30) + theta) * dist +  cos(theta)*0.5 + rx

                s_y[0] =  sin( radians(-30) + theta) * dist +  sin(theta)*0.5 + ry
                s_y[1] = sin( radians(0) + theta) * dist +  sin(theta)*0.5 + ry
                s_y[2] = sin( radians(30) + theta) * dist +  sin(theta)*0.5 + ry
                
            #find intersections
            for i in range(0, 3) :
                li_x = found_odds(rx , s_x[i])
                li_y = found_odds(ry , s_y[i])
                
                print(i, ': ',li_x, li_y)
                #intersect
                c = np.polyfit( [cos(theta)*0.5 + rx , s_x[i] ] , [sin(theta)*0.5 + ry , s_y[i]] , 1)

                #parede 0 - frente
                for j in range(0, len(li_x)):
                    if li_x[j] in ix : #nao tenho a certeza se nao terei de tirar os ix quando nao e considerado parede da frente TODO
                        continue
                    tmp_y = c[0] * li_x[j] + c[1]
                    if (tmp_y > min(ry, s_y[i]) and tmp_y < max(ry, s_y[i]) ):
                        #calcular trust aprox
                        tmp_y = find_Near_Odd(tmp_y)
                        d = hypot(li_x[j] - rx, tmp_y - ry) - 0.5
                        trust = translate(d)
                        g_y = divide(li_x[j]) + 15
                        g_x = abs(divide(tmp_y) - 8)
                        if li_x[j] < 0 :
                            wall_index = 2
                        else:
                            wall_index = 0
                        map_.updateWall(wall_index, (g_x, g_y) , -CONSTANT/3* trust)
                        print("ix: ", li_x[j],  "| iy : ", tmp_y , " | sx, sy" , (s_x[i], s_y[i]) )
                        print(wall_index, (g_x, g_y) , -CONSTANT/3* trust)

                #parede 1 - esquerda
                for j in range(0, len(li_y)):
                    if li_y[j] in iy : #nao tenho a certeza se nao terei de tirar os ix quando nao e considerado parede da frente TODO
                        continue
                    tmp_x = (li_y[j] - c[1] )/c[0]
                    if (tmp_x > min(rx, s_x[i]) and tmp_x < max(rx, s_x[i]) ):
                        #calcular trust aprox
                        tmp_x = find_Near_Odd(tmp_x)
                        d = hypot(li_y[j] - ry, tmp_x - rx) - 0.5
                        trust = translate(d)
                        g_y = divide(tmp_x) + 15
                        g_x = abs(divide(li_y[j]) - 8)
                        if li_y[j] < 0 :
                            wall_index = 1
                        else:
                            wall_index = 3
                        map_.updateWall(wall_index, (g_x, g_y) , -CONSTANT/3* trust)
                        print("iy: ", li_y[j], "| ix : " , tmp_x)
                        print(wall_index, (g_x, g_y) , -CONSTANT/3* trust)

                    
            



class Buffer(object):
    """docstring for Buffer"""
    def __init__(self, length):
        
        self.buf = [0]*length

    def add(self, val):
        self.buf = [val] + self.buf[:-1]

    def get(self):
        return sorted(self.buf)[1] #dinamico TODO



if __name__ == "__main__":
    s = Sensors()
    mp = Maze(15, 29)
    g_y = int(0/2) + 15
    g_x = abs(int(0/2) - 8)
    print("init : ", g_x , g_y)
    for i in range(0, 11):
        s.add( 1/sqrt(3), 1/sqrt(3), 6, 0.5)
        s.add( 1/sqrt(3), 1/sqrt(3), 6, 0.5)
        s.add( 1/sqrt(3), 1/sqrt(3), 6, 0.5)
        #
        # s.add( 1/sqrt(3), 1/sqrt(3), 3, 0.5)
        # s.add( 1/sqrt(3), 1/sqrt(3), 3, 0.5)
        # s.add( 1/sqrt(3), 1/sqrt(3), 3, 0.5)
        s.update_map(mp, 0,0, radians(0 ) ) 


    print (mp.drawMaze())

    print (str(mp.maze[7][14]))
    print (mp.maze[8][14])
    print (mp.maze[8][15])
    print (mp.maze[9][15])

    # s.add( 1/sqrt(3), 1/sqrt(3), 3.8, 0.5)
    # s.add( 1/sqrt(3), 1/sqrt(3), 3.8, 0.5)
    # s.add( 1/sqrt(3), 1/sqrt(3), 3.8, 0.5)
    # s.update_map("map", 0,-2, 0)

