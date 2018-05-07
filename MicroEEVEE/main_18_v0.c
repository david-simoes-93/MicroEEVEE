#include "mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>



#include "MazeMap.h" //maze[][]
#include "AStar.h"

#include "DodgeObstacle.h"
#include "FollowTheBeacon.h"
#include "FollowTheWall.h"
#include "StopAtBeacon.h"
#include "Wander.h"
#include "Sensors.h"
#include "bluetooth_comm.h"


/* --------- Functions -------------*/
void reactive_decide();

double get_next_dir(int sx, int sy);

bool compare_points(int * p, int*q){
    if(p[0] == q[0] && p[1]== q[1]) return true;
    return false;
}

void get_point_from_list();

/* --------- Variables -------------*/

int count_beacon;

int beaconPoint[2]; //in cm
int old_beaconPoint[2];
double dir_beacon, go_to_dir;
bool recalculate;

int lastBeh;

int paths[cols][rows][2]; //in cells

int path_list[100][2]; //in cells
int path_length = 0;


/* --------- Main -------------*/

int main(void) {
    /*init*/
    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);
    setRobotPos(0, 0, 0);

    /*wait until press start*/
    printf("Press start to continue\n");
    while (!startButton());

    //lights out
    leds(0x00);

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
    int c = 0;

    //int x, y;
    //double t;

    recalculate = true;
    followPoints = 0; //only true when beacon is reached
    bool execute_followPoints_ = true;
    /*-------------------*/

    while (!stopButton()) {

        // Fill in "analogSensors" structure
        readAnalogSensors();

        // Read ground sensor
        int groundSensor = readLineSensors(70);
        printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d\n", analogSensors.obstSensLeft,
               analogSensors.obstSensFront, analogSensors.obstSensRight, analogSensors.batteryVoltage);

        gndVals[c] = groundSensor;	c=(c+1)%5; //buffer

        /* Track robot position and orientation */

        getRobotPos_int(&x, &y, &t);

        //update map
        printf("update map...\n");
        update_map(x, y, analogSensors.obstSensLeft, analogSensors.obstSensFront, analogSensors.obstSensRight,t);
        printf("update map\n");

        //Find beacon
        if ( servoControl() ){ //found beacon
            //dir_beacon = beaconDir;
            /*calculate beacon*/
            old_beaconPoint[0] = beaconPoint[0];
            old_beaconPoint[1] = beaconPoint[1];
            int tmp[2] = {x,y};

            //add new Line and get Avg Beacon Point
            if (getDirectionTarget(tmp, dir_beacon, beaconPoint) ){ //i have a beacon point at beaconPoint
                if (! compare_points(old_beaconPoint, beaconPoint)) recalculate = true;
                execute_followPoints_ = true;
            }
            else{ //no intersect was made, only have direction!
                //go_to_dir = beaconDir;
                execute_followPoints_ = false;
            }
        }



        //get my cell coords
        int tmp[2];
        get_cell_index_from_gps_coords(x, y, tmp);
        int sx = tmp[0];
        int sy = tmp[1];

        if(recalculate){
            //recalculate new path
            if(followPoints){ //beacon was found --> go Home
                tmp[0] = 7; //in cells
                tmp[1] = 7;
            }
            else{
                get_cell_index_from_gps_coords(beaconPoint[0], beaconPoint[1], tmp);
            }
            int gx = tmp[0];
            int gy = tmp[1];

            if( astar(rows, cols, sx,sy,gx, gy, paths)){
                printf("Astar search %d\n", true);
                while (true) { //get Path_list
                    path_list[path_length][0] = paths[gx][gy][0];
                    path_list[path_length][1] = paths[gx][gy][1];
                    ++path_length;

                    printf("%d %d; ", paths[gx][gy][0], paths[gx][gy][1]);
                    int prev_x = paths[gx][gy][0], prev_y = paths[gx][gy][1];
                    gx = prev_x;
                    gy = prev_y;
                    if (gx == sx && gy == sy)
                        break;
                }
                //update lastPointx,y
                get_point_from_list();
                //beaconDir = get_next_dir(sx, sy); //in cells

            }
            else{
                printf("Cannot find path to goal (%d, %d) \n", gx, gy);
                break;
            }

            recalculate = false;
        }


        if (sx == path_list[path_length-1][0] && sy == path_list[path_length-1][1] ){
            if(followPoints && sx == 0 && sy == 0) {
                //reach Home
                printf("Found Home");
                break;
            }
            path_length = path_length-1;
            get_point_from_list();
            //beaconDir = get_next_dir(sx, sy);
        }

        if(execute_followPoints_){
            followPoints_(); //update beaconDir every iteration
        }
        //else beaconPoint from servoControl is executed
        reactive_decide(); //ftb executes with beaconDir value


    }

    print_map();


    disableObstSens();
    setVel2(0,0);
    int frequency = 4; //Hz
    while(true){
        leds(0x00);
        wait(1/frequency/10);
        leds(0xFF);
        wait(1/frequency/10);
    }
    return 0;

}
//update LastPointX LastPointY
void get_point_from_list(){
    int target_path_point_index = path_length-1;
    if (path_length > 1)
        target_path_point_index = path_length-2;

    get_middle_coords_from_index(path_list[target_path_point_index][0], path_list[target_path_point_index][1], &lastPointX, &lastPointY);

}

// dir in DEGREES
double get_next_dir(int sx, int sy){
    // define beaconDir
    int target_path_point_index = path_length-1;
    if (path_length > 1)
        target_path_point_index = path_length-2;
    return -normalizeAngle(get_target_dir(sx, sy, t,
                          path_list[target_path_point_index][0], path_list[target_path_point_index][1])) * (180 / PI);
}

void reactive_decide() {
    //printf("\tLeft %2.2f; Front %2.2f; Right %2.2f\n", obstValLeft, obstValFront, obstValRight);
    //printf("X= %f Y= %f Compass= %f\n", x, y, compass);
    prep_ftw();

    if(sab_isPossible()){
        //all light on
        leds(0xFF);
        recalculate = true; //now go home
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

