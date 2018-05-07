#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

//#include "Sensors.h"

#include "MazeMap.h" //maze[][]


double get_next_dir(int sx, int sy);

int count_beacon;

int beaconPoint[2]; //in cm
int old_beaconPoint[2];
double dir_beacon;
bool recalculate;

bool compare_points(int * p, int*q){
    if(p[0] == q[0] && p[1]== q[1]) return true;
    return false;
}


int paths[w][h][2]; //in cells

int path_list[100][2]; //in cells
int path_length = 0;

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

    recalculate = true;

    /*-------------------*/

    while (!stopButton()) {

        // Fill in "analogSensors" structure
        readAnalogSensors();

        // Read ground sensor
        int groundSensor = readLineSensors(70);

        gndVals[c] = groundSensor;	c=(c+1)%5; //buffer

        /* Track robot position and orientation */

        getRobotPos_int(&x, &y, &t);

        //update map
        update_map(x, y, analogSensors.obstSensLeft, analogSensors.obstSensFront, analogSensors.obstSensRight,t);



        if ( servoControl() ){
            dir_beacon = beaconDir;
            /*calculate beacon*/
            old_beaconPoint[0] = beaconPoint[0];
            old_beaconPoint[1] = beaconPoint[1];
            if (getDirectionTarget({x,y}, dir_beacon, &beaconPoint) ){ //i have a beacon point at beaconPoint
                if (! compare_points(&old_beaconPoint, &beaconPoint)) recalculate = true;
            }
            else{
                go_to_dir = dir_beacon;
            }
        }

        int sx = get_cell_coords_from_gps_coords(x);
        int sy = get_cell_coords_from_gps_coords(y);

        if(recalculate){
            //recalculate new path
            int gx = get_cell_coords_from_gps_coords(beaconPoint[0]);
            int gy = get_cell_coords_from_gps_coords(beaconPoint[1]);

            if( astar(rows, cols, sx,sy,gx, gy, &paths)){
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
                go_to_dir = get_next_dir(sx, sy); //in cells

            }
            else{
                printf("Cannot find path to goal (%d, %d) cm\n", beaconPoint[0]/100, beaconPoint[1]/100);
                break;
            }

            recalculate = false;
        }

        if (sx == path_list[path_length-1][0] && sy == path_list[path_length-1][1] ){
            path_length = path_length-1;
            go_to_dir = get_next_dir(sx, sy);
        }





        //decide what to do
        //decide();

    }



    disableObstSens();
    setVel2(0,0);


}


double get_next_dir(int sx, int sy){
    // define beaconDir
    int target_path_point_index = path_length-1;
    if (path_length > 1)
        target_path_point_index = path_length-2;

    return get_target_dir(sx, sy, t,
                          path_list[target_path_point_index][0], path_list[target_path_point_index][1]);
}