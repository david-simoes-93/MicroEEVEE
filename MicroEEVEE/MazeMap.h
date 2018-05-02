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

struct Cell
{
	/* data */
	struct Wall* north;
	struct Wall* south;
	struct Wall* east;
	struct Wall* west;
	struct Wall* walls[4] = {north, south, west, east};
	int coords[2];
	//uint_8 visited = 0;
};


struct Wall
{
	int weight= 0;
	int wall = 0; //wall true, no_wall, wall_confirmed --> mask
	float line[2][2];
	
};
struct Cell maze[rows][cols];

/*Functions of Maze*/

/*init map*/
void init();

/*ignora os valores acima de 60 cm: demasiado ruido*/
void update_map(double x, double y, double left, double front, double right ,double compass, int ground );

/*adiciona o valor*/
void update_single_sensor(double sensor_val, struct Cell[8], double sensor_positions[3][2] ,  double sensor_pos_in_eevee[2]);

/*mapeia o peso de confiança para dar a parede*/
int map_weight(double dist);


/*index of wall according to heading*/
int wall_index(double heading);


/*Functions of Wall*/
void weight(struct Wall* w, int val);

void confirm_no_wall(struct Wall* w);

bool confirmed_no_wall(struct Wall* w);
bool no_wall(struct Wall* w);
bool wall(struct Wall* w);

/* a* to find path*/
