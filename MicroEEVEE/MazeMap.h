#define  pcompile

#ifdef pcompile
#include "mr32.h"
#else
#include <stdio.h>
#define min(X, Y) (((X) <= (Y)) ? (X) : (Y))
#define max(X,Y) (((X) >= (Y)) ? (X) : (Y))
#define bool int
#define true 1
#define false 0
#endif

#include <stdlib.h>
#include <math.h>

#ifndef MAZE_MAP_H
#define MAZE_MAP_H

#define n_cells 8
#define rows (n_cells*2-1)
#define cols (n_cells*2-1)
#define m_point 22.5 // 45/2

/*index*/
#define north 0
#define south 1
#define west 2
#define east 3

#define pi_over_6 0.52359877559
#define pi_over_4 0.78539816339
#define pi_over_2 1.57079632679

/*is_wall value*/
#define max_val 255
#define min_val -255

struct Cell {
    /* data */
    struct Wall* north_wall;
    struct Wall* south_wall;
    struct Wall* east_wall;
    struct Wall* west_wall;
    struct Wall* walls[4]; // = {north_wall, south, west, east};
    int coords[2];
    int explored;
    //uint_8 visited = 0;
};


#define wall_mask         0b00000001
#define no_wall_mask      0b00000010
#define wall_conf_mask    0b00000101
#define no_wall_conf_mask 0b00001010

#define sensor_cutoff_point 4500
#define max_dist_threshold  6000

struct Wall {
    double weight;
    int wall; //is_wall true, is_no_wall, wall_confirmed --> mask
    int line[2][2];

};
struct Cell maze[cols][rows];

/*Functions of Maze*/

/*init map*/
void init_maze();


void get_cell_index_from_gps_coords(int x, int y, int *t);
// returns dot product of "v" and "w"
double dist2(int* v, int* w);

// returns euclidian distance between "v" and "w"
double dist(int* v,int* w);


// returns the square of the distance of point "p" to line segment between "v" and "w"
double dist_to_segment_squared(int* p, int* v, int* w);

// returns distance of point "p" to line segment between "v" and "w"
double dist_to_line_segment(int* p, int* v,  int* w);

/*ignora os valores acima de 60 cm: demasiado ruido*/
// compass in RADIANS
void update_map(int my_x, int my_y, int left_sensor, int front_sensor, int right_sensor, double compass);

void print_map();

/*adiciona o valor*/
void update_single_sensor(int sensor_val, struct Cell nearby_cells[9], int sensor_positions[2],
                          int sensor_pos_in_eevee[2]);

int intersects(int *AB, int* CD, int *PQ, int* RS) ;

/* returns desired dir based on target point */
double get_target_dir(int, int, double, int, int);

double trust_based_on_distance(int dist);

void get_middle_coords_from_index(int x, int y, int *px, int *py);
/*Functions of Wall*/
void weigh_wall(struct Wall *w, double val) ;


void confirm_no_wall(struct Wall *w);

void confirm_wall(struct Wall *w) ;

int is_confirmed_no_wall(struct Wall *w) ;

int is_no_wall(struct Wall *w);

int is_wall(struct Wall *w) ;


// add a new beacon line, mark a new beacon point and return avg beacon point
bool getDirectionTarget(int* curr, double dir, int *beaconPoint) ;

bool getBeaconAvgPoint(int *bp);

bool setUniqueBeaconPoint(int px, int py);


/* a* to find path*/
#endif


