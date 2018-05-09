import socket
import time
from croblink import CRobLink
from croblink import CRobLinkAngs
from math import cos, sin, sqrt, pow

from sensors import *
from mapgrid import *
from position import *

sensors = Sensors()
pos = Position()
map_ = Maze()

def wander():
    if cif.measures.irSensor[0]> 5.0\
        or cif.measures.irSensor[1]> 5.0\
        or cif.measures.irSensor[2]> 5.0\
        or cif.measures.irSensor[3]> 5.0:
        #    print "Rotate"
        cif.driveMotors(-0.1,+0.1)
    elif cif.measures.irSensor[1]> 0.7:
        cif.driveMotors(0.1,0.0)
    elif cif.measures.irSensor[2]> 0.7:
        cif.driveMotors(0.0,0.1)
    else:
        #    print "Go"
        cif.driveMotors(0.1,0.1)




if __name__ == "__main__":

    cif=CRobLink("CatFood",3,"localhost")
    if cif.status!=0:
        print "Connection refused or error"
        quit()

    RUN=1
    WAIT=2
    RETURN=3
    state = RUN

    while 1:
        #print "READ"
        cif.readSensors()
        #print "IRS "+ str(cif.measures.irSensor[0]) +" " + str(cif.measures.irSensor[1])\
        #         +" "+ str(cif.measures.irSensor[2]) +" "+ str(cif.measures.irSensor[3])
        #print "Drive"
        if state==RUN:
            if cif.measures.visitingLed==1:
                state=WAIT
            if cif.measures.ground==0:
                cif.setVisitingLed(1);
            wander()
        elif state==WAIT:
            cif.setReturningLed(1)
        if cif.measures.visitingLed==1:
            cif.setVisitingLed(0)
        if cif.measures.returningLed==1:
            state=RETURN
        cif.driveMotors(0.0,0.0)
        elif state==RETURN:
            if cif.measures.visitingLed==1:
                cif.setVisitingLed(0)
            if cif.measures.returningLed==1:
                cif.setReturningLed(0)
        wander()