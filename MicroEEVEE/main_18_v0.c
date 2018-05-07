#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "DodgeObstacle.h"
#include "FollowTheBeacon.h"
#include "FollowTheWall.h"
#include "StopAtBeacon.h"
#include "Wander.h"
//#include "Sensors.h"
#include "MazeMap.h"

void reactive_decide();
int lastBeh;

int count_beacon;
int beaconPoint[2];
double dir_beacon;

int main(void) {
    /*init*/
    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    setRobotPos(0, 0, 0);

    /*wait until press start*/
    printf("Press start to continue\n");
    while (!startButton());

    enableObstSens();
    // Fill in "analogSensors" structure
    int i;
    for (i = 0; i < 7; ++i) {
        readAnalogSensors();
        //groundSensor = readLineSensors(0);
    }

    /*variables*/
    count_beacon = 0;
    int count_ticks;
    count_ticks = 0;

    init_maze();

    int x, y;
    double t;
    /*-------------------*/

    while (!stopButton()) {
        waitTick20ms();

        // Fill in "analogSensors" structure
        readAnalogSensors();
        int groundSensor = readLineSensors(70);
        printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d\n", analogSensors.obstSensLeft,
               analogSensors.obstSensFront, analogSensors.obstSensRight, analogSensors.batteryVoltage);

        //gndVals[c] = groundSensor;	c=(c+1)%5; //buffer

        /* Track robot position and orientation */
        getRobotPos_int(&x, &y, &t);

        //update map
        printf("update map...\n");
        update_map(x, y, analogSensors.obstSensLeft, analogSensors.obstSensFront, analogSensors.obstSensRight,t);
        printf("update map\n");

        //reactive_decide();
        setVel2(0,0);
        printf("decide\n");

        print_map();
        printf("print\n");
        int i;
        for(i=0; i<100; i++)
            waitTick20ms();
    }

    /*calculate beacon*/
    /*if (getDirectionTarget({x,y}, dir_beacon, &beaconPoint) ){

    }*/

    disableObstSens();
    setVel2(0,0);
}


void reactive_decide() {
    //printf("\tLeft %2.2f; Front %2.2f; Right %2.2f\n", obstValLeft, obstValFront, obstValRight);
    //printf("X= %f Y= %f Compass= %f\n", x, y, compass);
    prep_ftw();

    if(sab_isPossible()){
        sab_execute();
        if(lastBeh!=0){
            printf("\tExecuting behaviour %s\n", sab_getName());
            lastBeh=0;
        }
    }
    else if(ftw_isPossible()){
        ftw_execute();
        if(lastBeh!=1){
            printf("\tExecuting behaviour %s\n", ftw_getName());
            lastBeh=1;
        }
    }
    else if(ftb_isPossible()){
        ftb_execute();
        if(lastBeh!=2){
            printf("\tExecuting behaviour %s\n", ftb_getName());
            lastBeh=2;
        }
    }
    else if(do_isPossible()){
        do_execute();
        if(lastBeh!=3){
            printf("\tExecuting behaviour %s\n", do_getName());
            lastBeh=3;
        }
    }
    else if(wan_isPossible()){
        wan_execute();
        if(lastBeh!=4){
            printf("\tExecuting behaviour %s\n", wan_getName());
            lastBeh=4;
        }
    }
}


