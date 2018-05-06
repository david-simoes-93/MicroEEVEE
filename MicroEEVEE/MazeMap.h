#include <stdio.h>

#ifndef MAZE_MAP_H
#define MAZE_MAP_H

#define n_cells 8
#define rows n_cells*2-1
#define cols n_cells*2-1
#define m_point 22.5 // 45/2

/*index*/
#define north 0
#define south 1
#define west 2
#define east 3

/*wall value*/
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


#define wall_mask         = 0b00000001
#define no_wall_mask      = 0b00000010
#define wall_conf_mask    = 0b00000101
#define no_wall_conf_mask = 0b00001010

struct Wall {
    int weight;
    int wall; //wall true, no_wall, wall_confirmed --> mask
    float line[2][2];

};
struct Cell maze[rows][cols];

/*Functions of Maze*/

/*init map*/
void init_maze();

/*ignora os valores acima de 60 cm: demasiado ruido*/
void update_map(float, float, float, float, float, float);

/*adiciona o valor*/
void update_single_sensor(float sensor_val, struct Cell *, float **, float *);

/*mapeia o peso de confiança para dar a parede*/
int map_weight(float dist);

/* prints current map to console */
void print_map();


/*index of wall according to heading*/
int wall_index(float heading);


/*Functions of Wall*/
void weight(struct Wall *w, int val);

void confirm_no_wall(struct Wall *w);

bool confirmed_no_wall(struct Wall *w);

bool no_wall(struct Wall *w);

bool wall(struct Wall *w);

/* a* to find path*/
