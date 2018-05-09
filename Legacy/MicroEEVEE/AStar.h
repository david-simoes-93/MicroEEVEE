#ifndef AStar_h
#define AStar_h

#include <stdlib.h>
#include "MazeMap.h"

#ifdef pcompile
#include "mr32.h"
#else
#include <stdio.h>
#endif




int astar(const int h, const int w,
           const int start_x, const int start_y, const int goal_x, const int goal_y,
           int paths[w][h][2]);

float dist_manhattan(int i0, int j0, int i1, int j1);

// represents a single pixel
struct  Node {
    int x, y;    // index in the flattened grid
    float cost;  // cost of traversing this pixel
    //Node(int i, float c) : idx(i),cost(c) {}
};


#endif
