#include "mr32.h"
#include <stdlib.h>


#include "MazeMap.h" //maze[][]
#include "AStar.h"

#include "DodgeObstacle.h"
#include "FollowTheBeacon.h"
#include "FollowTheWall.h"
#include "StopAtBeacon.h"
#include "Wander.h"
#include "Sensors.h"


/* --------- Functions -------------*/
void reactive_decide();

double get_next_dir(int sx, int sy);

bool compare_points(int *p, int *q) {
    if (p[0] == q[0] && p[1] == q[1])
        return true;
    return false;
}

void get_point_from_list();

/* --------- Variables -------------*/
int beaconPoint[2]; //in cm
int old_beaconPoint[2];
double dir_beacon, go_to_dir;
bool should_recalculate_astar;
int lastBeh;
int paths[cols][rows][2]; //in cells
//int path_list[100][2]; //in cells
//int path_length = 0;

int path_list[9][2] = {{12,10},{12,9}, {12,8},{12,7},{11,7}, {10,7},{9,7},{8,7}, {8,8}};
int path_length = 9;
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
    init_maze();
    int ground_sensor_buffer_index = 0;
    int home[2]={0,0};
    should_recalculate_astar = false;
    returning_home = 0; //only true when beacon is reached
    bool follow_astar_path = false;

    /*-------------------*/

    get_point_from_list();
    while (!stopButton()) {
        // Fill in "analogSensors" structure
        readAnalogSensors();
        obstValLeft = analogSensors.obstSensLeft / 100 -7;
        obstValFront = analogSensors.obstSensFront / 100 - 7 ;
        obstValRight = analogSensors.obstSensRight / 100 -7;

        // ground
        int groundSensor = readLineSensors(70);
        ground_sensor_buffer[ground_sensor_buffer_index] = groundSensor;
        ground_sensor_buffer_index = (ground_sensor_buffer_index + 1) % 5;

        printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d\n", analogSensors.obstSensLeft,
               analogSensors.obstSensFront, analogSensors.obstSensRight, analogSensors.batteryVoltage);

        /* Track robot position and orientation */
        getRobotPos_int(&x, &y, &t); //mm*10
        int my_pos[2] = {x, y};

        //update map
        //printf("update map\n");
        update_map(x, y, analogSensors.obstSensLeft, analogSensors.obstSensFront, analogSensors.obstSensRight, t);




        //get my cell coords
        int my_cell_index[2];
        get_cell_index_from_gps_coords(x, y, my_cell_index);
        printf("mycell (%d %d), %d cm, %d cm \n" , my_cell_index[0], my_cell_index[1], x/100, y/100);
        int sx = my_cell_index[0];
        int sy = my_cell_index[1];

        follow_astar_path = true;
        if (follow_astar_path) {
            //printf("ent of path, @%d %d\n", my_cell_index[0], my_cell_index[1]);
            if (sx == path_list[path_length-1][0] && sy == path_list[path_length-1][1]) { //10 cm

                //reach Home
                if (returning_home && dist(my_pos,home)<1000) {
                    printf("Found Home");
                    break;
                }

                path_length = path_length - 1;
                get_point_from_list();

            }

            //printf("follow astar\n");
            follow_target_point(); //update beaconDir every iteration
        }

        //printf("reactive\n");
        //else beaconPoint from servoControl is executed
        reactive_decide(); //ftb executes with beaconDir value
        //setVel2(0,0);
        //print_map();
    }

    print_map();

    disableObstSens();
    setVel2(0, 0);
    while (true) {
        leds(0x00);
        wait(5);
        leds(0xFF);
        wait(5);
    }
    return 0;

}

//update LastPointX LastPointY
void get_point_from_list() {
    int target_path_point_index = path_length - 1;
    if (path_length > 1)
        target_path_point_index = path_length - 2;
    printf("get new point , (%d %d)\n", path_list[target_path_point_index][0], path_list[target_path_point_index][1] );
    get_middle_coords_from_index(path_list[target_path_point_index][0], path_list[target_path_point_index][1],
                                 &target_point_x, &target_point_y);

}

// dir in DEGREES
double get_next_dir(int sx, int sy) {
    // define beaconDir
    int target_path_point_index = path_length - 1;
    //if (path_length > 1)
    //    target_path_point_index = path_length - 2;
    return -normalizeAngle(get_target_dir(sx, sy, t,
                                          path_list[target_path_point_index][0],
                                          path_list[target_path_point_index][1])) * (180 / PI);
}

void reactive_decide() {
    //printf("\tLeft %2.2f; Front %2.2f; Right %2.2f\n", obstValLeft, obstValFront, obstValRight);
    //printf("X= %f Y= %f Compass= %f\n", x, y, compass);
    prep_ftw();

    if (sab_isPossible()) {
        //all light on
        leds(0xFF);
        should_recalculate_astar = true; // now go home
        sab_execute();
        if (lastBeh != 0) {
            printf("\tExecuting behaviour %s\n", sab_getName());
            lastBeh = 0;
        }
    } else if (ftw_isPossible()) {
        ftw_execute();
        if (lastBeh != 1) {
            printf("\tExecuting behaviour %s\n", ftw_getName());
            lastBeh = 1;
        }
    } else if (ftb_isPossible()) {
        ftb_execute();
        if (lastBeh != 2) {
            printf("\tExecuting behaviour %s\n", ftb_getName());
            lastBeh = 2;
        }
    } else if (do_isPossible()) {
        do_execute();
        if (lastBeh != 3) {
            printf("\tExecuting behaviour %s\n", do_getName());
            lastBeh = 3;
        }
    } else if (wan_isPossible()) {
        wan_execute();
        if (lastBeh != 4) {
            printf("\tExecuting behaviour %s\n", wan_getName());
            lastBeh = 4;
        }
    }
}

