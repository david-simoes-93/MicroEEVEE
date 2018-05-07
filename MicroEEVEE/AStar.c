#include "AStar.h"

float dist_manhattan(int i0, int j0, int i1, int j1) {
    return abs(i0 - i1) + abs(j0 - j1);
}

#ifndef pcompile

int main(int argc, char **argv) {
    int h = rows, w = cols;
    int sx = 1, sy = 1;
    int gx = 10, gy = 10;

    int i, j;
    int ***paths = (int ***) malloc(w * sizeof(int **));
    for (i = 0; i < w; i++) {
        paths[i] = (int **) malloc(h * sizeof(int *));
        for (j = 0; j < h; j++) {
            paths[i][j] = (int *) malloc(2 * sizeof(int));
        }
    }
    init_maze();

    int x;
    for (x = 0; x < 5; ++x) {
        weigh_wall(maze[x][5].south_wall, 1000);
    }
    print_map();

    int worked = astar(h, w, sx, sy, gx, gy, paths);
    printf("%d\n", worked);
    int path_length = 0;
    int path_list[100][2];
    while (worked) {
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

    if(worked) {
        // define beaconDir
        int target_path_point_index = path_length-1;
        if (path_length > 1)
            target_path_point_index = path_length-2;

        float dir = get_target_dir(sx, sy, 0,
                                   path_list[target_path_point_index][0], path_list[target_path_point_index][1]);
        printf("%f\n",dir);

    }

    return 0;
}

#endif

// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): int[w][h][2], stores for each node its previous node in the path
int astar(
        const int h, const int w,
        const int start_x, const int start_y, const int goal_x, const int goal_y,
        int ***paths) {

    int N_neigh = 4;
    const float INF = 10000;

    struct Node start_node = {.x = start_x, .y = start_y, .cost = 0.};
    struct Node goal_node = {.x = goal_x, .y=goal_y, .cost = 0.};

    float costs[w][h];
    int i, j;
    for (i = 0; i < h; ++i)
        for (j = 0; j < w; ++j)
            costs[j][i] = INF;
    costs[start_x][start_y] = 0.;

    int length_nodes_to_visit = 1;
    struct Node nodes_to_visit[h * w];
    nodes_to_visit[0] = start_node;

    int nbrs[N_neigh][2];

    int solution_found = 0;
    while (length_nodes_to_visit > 0) {
        struct Node cur = nodes_to_visit[0];

        if (cur.x == goal_node.x && cur.y == goal_node.y) {
            solution_found = 1;
            break;
        }

        // pop
        for (i = 1; i < length_nodes_to_visit + 1; ++i) {
            nodes_to_visit[i - 1].x = nodes_to_visit[i].x;
            nodes_to_visit[i - 1].y = nodes_to_visit[i].y;
            nodes_to_visit[i - 1].cost = nodes_to_visit[i].cost;
        }
        --length_nodes_to_visit;

        // check bounds and find neighbors
        if (cur.y - 1 >= 0 && is_wall(maze[cur.x][cur.y].north_wall) == 0) {
            nbrs[0][0] = cur.x;
            nbrs[0][1] = cur.y - 1;
        } else {
            nbrs[0][0] = -1;
            nbrs[0][1] = -1;
        }
        if (cur.y + 1 < h && is_wall(maze[cur.x][cur.y].south_wall) == 0) {
            nbrs[1][0] = cur.x;
            nbrs[1][1] = cur.y + 1;
        } else {
            nbrs[1][0] = -1;
            nbrs[1][1] = -1;
        }
        if (cur.x - 1 >= 0 && is_wall(maze[cur.x][cur.y].west_wall) == 0) {
            nbrs[2][0] = cur.x - 1;
            nbrs[2][1] = cur.y;
        } else {
            nbrs[2][0] = -1;
            nbrs[2][1] = -1;
        }
        if (cur.x + 1 < w && is_wall(maze[cur.x][cur.y].east_wall) == 0) {
            nbrs[2][0] = cur.x + 1;
            nbrs[2][1] = cur.y;
        } else {
            nbrs[2][0] = -1;
            nbrs[2][1] = -1;
        }

        float heuristic_cost;
        for (i = 0; i < N_neigh; ++i) {
            if (nbrs[i][0] >= 0 && nbrs[i][1] >= 0) {
                // the sum of the cost so far and the cost of this move
                float new_cost = costs[cur.x][cur.y] + 1;
                if (new_cost < costs[nbrs[i][0]][nbrs[i][1]]) {
                    heuristic_cost = dist_manhattan(nbrs[i][0], nbrs[i][1], goal_x, goal_y);

                    // paths with lower expected cost are explored first
                    float priority = new_cost + heuristic_cost;
                    //struct Node node_to_visit = {.x = nbrs[i][0], .y = nbrs[i][1], .cost = priority};
                    int insert_index;
                    for (insert_index = 0; insert_index < length_nodes_to_visit; ++insert_index) {
                        if (nodes_to_visit[insert_index].cost > priority)
                            break;
                    }
                    for (j = length_nodes_to_visit; j > insert_index; --j) {
                        nodes_to_visit[j] = nodes_to_visit[j - 1];
                    }
                    nodes_to_visit[insert_index].x = nbrs[i][0];
                    nodes_to_visit[insert_index].y = nbrs[i][1];
                    nodes_to_visit[insert_index].cost = priority;
                    ++length_nodes_to_visit;

                    costs[nbrs[i][0]][nbrs[i][1]] = new_cost;
                    paths[nbrs[i][0]][nbrs[i][1]][0] = cur.x;
                    paths[nbrs[i][0]][nbrs[i][1]][1] = cur.y;
                }
            }
        }
    }

    return solution_found;
}
