#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

//#include "Sensors.h"

#include "MazeMap.h" //maze[][]

bool recalculate_path(int *);


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


    /*-------------------*/

    while (!stopButton()) {
        count_ticks++;
        waitTick20ms();
        switch (count_ticks) {
            case 1 :
                // tick 40ms
                // Fill in "analogSensors" structure
                readAnalogSensors();

                // Read ground sensor
                int groundSensor = readLineSensors(70);

                //gndVals[c] = groundSensor;	c=(c+1)%5; //buffer

                /* Track robot position and orientation */
                int x, y;
                double t;
                getRobotPos_int(&x, &y, &t);

                //update map
                update_map(x, y, analogSensors.obstSensLeft, analogSensors.obstSensFront, analogSensors.obstSensRight,
                           t);


                break;

            case 2:
                //recalculate_path();

                count_ticks = 0;
                break;
        }


    }

    /*calculate beacon*/
    if (getDirectionTarget({x,y}, dir_beacon, &beaconPoint) ){

    }

    //decide what to do
    decide();

    disableObstSens();
    setVel2(0,0);


}


