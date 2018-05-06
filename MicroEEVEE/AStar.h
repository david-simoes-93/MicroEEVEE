#ifndef AStar_h
#define AStar_h

#include <stdlib.h>

bool astar(const float* weights, const int h, const int w,
      const int start, const int goal, bool diag_ok,
      int* paths);

float l1_norm(int i0, int j0, int i1, int j1)

// represents a single pixel
struct  Node {

    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    //Node(int i, float c) : idx(i),cost(c) {}
};


#endif
