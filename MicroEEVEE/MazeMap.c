#include "MazeMap.h"

#define half_cell_width 2250 //22.5*100
#define cell_width 4500
/*init map*/

struct Wall walls_array[cols * rows * 2 + cols + rows];

void init_maze() {
    int i, j, walls_array_index = 0;
    for (i = 0; i < cols; i++) {
        for (j = 0; j < rows; j++) {
            int x = (i - n_cells) * cell_width;
            int y = (j - n_cells) * cell_width;

            maze[i][j].coords[0] = x;
            maze[i][j].coords[1] = y;

            maze[i][j].index[0] = i;
            maze[i][j].index[1] = j;

            // copy west neighbor's east is_wall
            if (i > 0) {
                maze[i][j].west_wall = maze[i - 1][j].east_wall;
            } else //unless we're on the left column
            {
                struct Wall *ptr_west = &walls_array[walls_array_index]; //(struct Wall *) malloc((sizeof(struct Wall)));
                walls_array_index++;
                maze[i][j].west_wall = ptr_west;

                ptr_west->weight = 0;
                ptr_west->wall = 0;
                ptr_west->line[0][0] = x - half_cell_width;
                ptr_west->line[0][1] = y - half_cell_width;
                ptr_west->line[1][0] = x - half_cell_width;
                ptr_west->line[1][1] = y + half_cell_width;
            }

            // always create the east is_wall
            struct Wall *ptr_east = &walls_array[walls_array_index]; //(struct Wall *) malloc((sizeof(struct Wall)));
            walls_array_index++;
            maze[i][j].east_wall = ptr_east;

            ptr_east->weight = 0;
            ptr_east->wall = 0;
            ptr_east->line[0][0] = x + half_cell_width;
            ptr_east->line[0][1] = y - half_cell_width;
            ptr_east->line[1][0] = x + half_cell_width;
            ptr_east->line[1][1] = y + half_cell_width;

            // copy north neighbor's south is_wall
            if (j > 0) {
                maze[i][j].north_wall = maze[i][j - 1].south_wall;
            } else // unless we're in the top row, where we have t actually create the damn is_wall
            {
                struct Wall *ptr_north = &walls_array[walls_array_index]; //(struct Wall *) malloc((sizeof(struct Wall)));
                walls_array_index++;
                maze[i][j].north_wall = ptr_north;

                ptr_north->weight = 0;
                ptr_north->wall = 0;
                ptr_north->line[0][0] = x - half_cell_width;
                ptr_north->line[0][1] = y - half_cell_width;
                ptr_north->line[1][0] = x + half_cell_width;
                ptr_north->line[1][1] = y - half_cell_width;
            }

            // always create bottom is_wall
            struct Wall *ptr_south = &walls_array[walls_array_index]; //(struct Wall *) malloc((sizeof(struct Wall)));
            walls_array_index++;
            maze[i][j].south_wall = ptr_south;

            ptr_south->weight = 0;
            ptr_south->wall = 0;
            ptr_south->line[0][0] = x - half_cell_width;
            ptr_south->line[0][1] = y + half_cell_width;
            ptr_south->line[1][0] = x + half_cell_width;
            ptr_south->line[1][1] = y + half_cell_width;

        }
    }

    for (i = 0; i < cols; i++) {
        for (j = 0; j < rows; j++) {
            maze[i][j].walls[north] = maze[i][j].north_wall;
            maze[i][j].walls[south] = maze[i][j].south_wall;
            maze[i][j].walls[west] = maze[i][j].west_wall;
            maze[i][j].walls[east] = maze[i][j].east_wall;

            if (i == 0) {
                confirm_wall(maze[i][j].west_wall);
            } else if (i == cols - 1) {
                confirm_wall(maze[i][j].east_wall);
            }

            if (j == 0) {
                confirm_wall(maze[i][j].north_wall);
            } else if (j == rows - 1) {
                confirm_wall(maze[i][j].south_wall);
            }
        }
    }

}

void print_map() {
    int x, y;
    for (x = 0; x < cols; x++) {
        if (is_wall(maze[x][0].north_wall))
            printf("__");
        else if (is_no_wall(maze[x][0].north_wall))
            printf("_ ");
        else
            printf("_*");
    }
    printf("\n");
    for (y = 0; y < rows; y++) {
        for (x = 0; x < cols; x++) {

            //printf("%d %d\n", x, y);
            if (is_wall(maze[x][y].west_wall))
                printf("|");
            else if (is_no_wall(maze[x][y].west_wall))
                printf(" ");
            else
                printf("?");

            if (is_wall(maze[x][y].south_wall))
                printf("_");
            else if (is_no_wall(maze[x][y].south_wall))
                printf(" ");
            else
                printf("*");
        }

        if (is_wall(maze[cols - 1][y].east_wall))
            printf("|");
        else if (is_no_wall(maze[cols - 1][y].east_wall))
            printf(" ");
        else
            printf("?");
        printf("\n");
    }
}


void get_cell_index_from_gps_coords(int x, int y, int *t) {
    // min cell is 1 and max cell is cols-2; this avoids agent being on border of maze
    // and having inexistnt neighbors on map functions
    t[0] = min(cols - 2, max((int) ((x * 1.0 + half_cell_width) / cell_width + n_cells), 1));
    t[1] = min(rows - 2, max((int) ((y * 1.0 + half_cell_width) / cell_width + n_cells), 1));
}

void get_middle_coords_from_index(int x, int y, int *px, int *py) {
    //t[0] = min(cols-1,max((int) (x / 4500 + (n_cells-1)+0.5),0));
    //t[1] = min(rows-1,max((int) (y / 4500 + (n_cells-1)+0.5),0));

    *px = (x - 0.5 - n_cells) * 4500;
    *py = (y - 0.5 - n_cells) * 4500;

}


// returns dot product of "v" and "w"
double dist2(int *v, int *w) {
    return (v[0] - w[0]) * (v[0] - w[0]) + (v[1] - w[1]) * (v[1] - w[1]);
}


// returns euclidian distance between "v" and "w"
double dist(int *v, int *w) {
    return sqrt(dist2(v, w));
}


// returns the square of the distance of point "p" to line segment between "v" and "w"
double dist_to_segment_squared(int *p, int *v, int *w) {
    double l2 = dist2(v, w);
    if (l2 == 0) { return dist2(p, v); }


    double t;
    t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2;
    t = max(0, min(1, t));

    int closest_point_in_line[2] = {v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])};
    return dist2(p, closest_point_in_line);
}


double get_target_dir(int myX, int myY, double myDir, int targetX, int targetY) {
    double meToTargetVector[2] = {targetX - myX, targetY - myY};
    double myDirectionVector[2] = {cos(myDir), sin(myDir)};

    double radianTargetDir =
            atan2(meToTargetVector[1], meToTargetVector[0]) - atan2(myDirectionVector[1], myDirectionVector[0]);

    return radianTargetDir;
}

// returns distance of point "p" to line segment between "v" and "w"
double dist_to_line_segment(int *p, int *v, int *w) {
    return sqrt(dist_to_segment_squared(p, v, w));
}

/*ignora os valores acima de 60 cm: demasiado ruido*/
// compass in RADIANS
void update_map(int my_x, int my_y, int left_sensor, int front_sensor, int right_sensor, double compass) {
    int my_pos[2] = {my_x, my_y};
    int my_cell_index[2];
    get_cell_index_from_gps_coords(my_x, my_y, my_cell_index);

    struct Cell my_cell = maze[my_cell_index[0]][my_cell_index[1]];
    my_cell.explored = 1;

    struct Cell nearby_cells[5] = {my_cell,
                                   maze[my_cell_index[0] + 1][my_cell_index[1]],
                                   maze[my_cell_index[0] - 1][my_cell_index[1]],
                                   maze[my_cell_index[0]][my_cell_index[1] + 1],
                                   maze[my_cell_index[0]][my_cell_index[1] - 1]};
    int min_left_sensor = max_dist_threshold, min_front_sensor = max_dist_threshold, min_right_sensor = max_dist_threshold;
    if (left_sensor < max_dist_threshold) min_left_sensor = left_sensor;
    if (front_sensor < max_dist_threshold) min_front_sensor = front_sensor;
    if (right_sensor < max_dist_threshold) min_right_sensor = right_sensor;

    double cos_compass = cos(compass);
    double sin_compass = sin(compass);

    int front_sensor_pos_in_eevee[2] = {my_pos[0] + (int) (300 * cos_compass),
                                        my_pos[1] + (int) (300 * sin_compass)};
    int front_sensor_dots[2];
    front_sensor_dots[0] = (int) (front_sensor_pos_in_eevee[0] + min_front_sensor * cos_compass);
    front_sensor_dots[1] = (int) (front_sensor_pos_in_eevee[1] + min_front_sensor * sin_compass);
    update_single_sensor(front_sensor, nearby_cells, front_sensor_dots, front_sensor_pos_in_eevee);

    if(na primeira parte da célula){
        int left_sensor_pos_in_eevee[2] = {my_pos[0] + (int) (500 * cos_compass - +600 * sin_compass),
                                           my_pos[1] + (int) (500 * sin_compass + +600 * cos_compass)};
        int left_sensor_dots[2];
        left_sensor_dots[0] = (int) (left_sensor_pos_in_eevee[0] + min_left_sensor * cos(compass - pi_over_4));
        left_sensor_dots[1] = (int) (left_sensor_pos_in_eevee[1] + min_left_sensor * sin(compass - pi_over_4));
        update_single_sensor(left_sensor, nearby_cells, left_sensor_dots, left_sensor_pos_in_eevee);

        int right_sensor_pos_in_eevee[2] = {my_pos[0] + (int) (500 * cos_compass - -600 * sin_compass),
                                            my_pos[1] + (int) (500 * sin_compass + -600 * cos_compass)};
        int right_sensor_dots[2];
        right_sensor_dots[0] = (int) (right_sensor_pos_in_eevee[0] + min_right_sensor * cos(compass + pi_over_4));
        right_sensor_dots[1] = (int) (right_sensor_pos_in_eevee[1] + min_right_sensor * sin(compass + pi_over_4));
        update_single_sensor(right_sensor, nearby_cells, right_sensor_dots, right_sensor_pos_in_eevee);
    }
    // confirm no walls where we are moving through
    // run through all neighbor cells
    int cell_index, wall_index;
    for (cell_index = 0; cell_index < 5; cell_index++) {
        // and 4 walls on each cell
        for (wall_index = 0; wall_index < 4; wall_index++) {
            struct Wall wall = *nearby_cells[cell_index].walls[wall_index];
            if (dist_to_line_segment(my_pos, wall.line[0], wall.line[1]) < 900)
                //printf("confirming no wall on cell (%d,%d), wall %d\n",nearby_cells[cell_index].index[0],
                //       nearby_cells[cell_index].index[1], wall_index);
                confirm_no_wall(&wall);
        }
    }
}

/*adiciona o valor*/

void update_single_sensor(int sensor_val, struct Cell nearby_cells[5], int sensor_positions[2],
                          int sensor_pos_in_eevee[2]) {

    printf("sensor at (%d,%d) hitting pos %d %d\n", sensor_pos_in_eevee[0], sensor_pos_in_eevee[1],
           sensor_positions[0], sensor_positions[1]);

    //printf("sensor at (%d,%d) hitting pos %d %d\n", sensor_pos_in_eevee[0], sensor_pos_in_eevee[1],
   //        sensor_positions[0], sensor_positions[1]);

    // if obstacle found
    //int possible_walls_size = 2 * 4;
    //int currently_weighted_walls = 0;
    //struct Wall *weighted_walls[possible_walls_size];
    int margin = 500;

    int wall_counter=0;
    struct Wall* walls_detected[5*4];

    int cell_index, wall_index;
    for (cell_index = 0; cell_index < 5; cell_index++) {
        if ((cell_index == 1 && is_wall(nearby_cells[0].east_wall)) ||
            (cell_index == 2 && is_wall(nearby_cells[0].west_wall)) ||
            (cell_index == 3 && is_wall(nearby_cells[0].south_wall)) ||
            (cell_index == 4 && is_wall(nearby_cells[0].north_wall))) {
            continue;
        }

        for (wall_index = 0; wall_index < 4; wall_index++) {
            struct Wall *wall = nearby_cells[cell_index].walls[wall_index];
            // sensor hitting wall within margin
            if(dist_to_line_segment(sensor_positions, wall->line[0],wall->line[1])<margin){
                walls_detected[wall_counter]=wall;
                wall_counter++;

             //   printf("\tDIST MARKING cell (%d,%d), wall %d\n",
             //          nearby_cells[cell_index].index[0], nearby_cells[cell_index].index[1], wall_index);
            //}
            //if(sensor_positions[0]>wall->line[0][0]-margin && sensor_positions[0]<wall->line[1][0]+margin
            //        && sensor_positions[1]>wall->line[0][1]-margin && sensor_positions[1]<wall->line[1][1]+margin){
            //    printf("\tMARKING cell (%d,%d), wall %d\n",
            //           nearby_cells[cell_index].index[0], nearby_cells[cell_index].index[1], wall_index);

                //weigh_wall(wall, 5);
            } // else if sensor beyond wall
            else if (intersects(wall->line[0], wall->line[1], sensor_pos_in_eevee, sensor_positions))
            {
              //  printf("\tCLEARING cell (%d,%d), wall %d\n",
              //         nearby_cells[cell_index].index[0], nearby_cells[cell_index].index[1], wall_index);
                weigh_wall(wall, -5);
            }
        }
    }

    for(wall_index=0; wall_index<wall_counter; ++wall_index){
        weigh_wall(walls_detected[wall_index],10.0/wall_counter);
    }

    /*
    if (sensor_val < sensor_cutoff_point)  // self.max_dist_threshold:
    {
        // check closest ray to any is_wall
        double ray_dists = max_dist_threshold;
        int min_cell_index = 0, min_wall_index = 0;
        struct Wall *closest_dot_cell_wall = NULL;

        int cell_index, wall_index;
        // run through all neighbor cells
        for (cell_index = 0; cell_index < 1; cell_index++) {    //5

            if ((cell_index == 1 && is_wall(nearby_cells[0].east_wall)) ||
                (cell_index == 2 && is_wall(nearby_cells[0].west_wall)) ||
                (cell_index == 3 && is_wall(nearby_cells[0].south_wall)) ||
                (cell_index == 4 && is_wall(nearby_cells[0].north_wall))) {
                continue;
            }
            // and 4 walls on each cell
            for (wall_index = 0; wall_index < 4; wall_index++) {
                //printf("\t@@(%d,%d) %d\n",nearby_cells[cell_index].coords[0], nearby_cells[cell_index].coords[1],wall_index);
                struct Wall *wall = nearby_cells[cell_index].walls[wall_index];
                // calculate distance of sensor point to the wall and save closest is_wall
                double d = dist_to_line_segment(sensor_positions, wall->line[0], wall->line[1]);
                if (d < ray_dists) {
                    ray_dists = d;
                    min_cell_index = cell_index;
                    min_wall_index = wall_index;
                    closest_dot_cell_wall = wall;
                    //printf("\tClosest wall from cell (%d,%d), wall %d, ptr %x, with dist %f\n", nearby_cells[cell_index].coords[0],
                    //       nearby_cells[cell_index].coords[1], wall_index, wall, ray_dists);
                    //printf("\t(%d,%d) to (%d,%d)\n",wall->line[0][0], wall->line[0][1], wall->line[1][0], wall->line[1][1]);

                }/*else{
                    printf("\t\tIgnoring wall from cell %d, wall %d, ptr %x, with dist %f\n", cell_index, wall_index, wall, d);
                    printf("\t\t(%d,%d) to (%d,%d)\n",wall->line[0][0], wall->line[0][1], wall->line[1][0], wall->line[1][1]);
                }* /
            }
        }
        // if that ray is actually close to a wall
        if (ray_dists<1000) {
            double trust_val = trust_based_on_distance(dist_to_line_segment(
                    sensor_pos_in_eevee, closest_dot_cell_wall->line[0], closest_dot_cell_wall->line[1]));
            printf("\tWeightin cell (%d,%d), wall %d with %f (was %f)\n",
                   nearby_cells[min_cell_index].index[0], nearby_cells[min_cell_index].index[1],
                   min_wall_index, trust_val, closest_dot_cell_wall->weight);
            //       closest_dot_cell_wall->line[0][0], closest_dot_cell_wall->line[0][1],
            //       closest_dot_cell_wall->line[1][0], closest_dot_cell_wall->line[1][1]);
            weigh_wall(closest_dot_cell_wall, trust_val);
            weighted_walls[0] = closest_dot_cell_wall;
            currently_weighted_walls++;
        }
    }

    // decrease all other is_wall intersections score
    int cell_index, wall_index;

    // run through all neighbor cells
    for (cell_index = 0; cell_index < 1; cell_index++) { //5
        // and 4 walls on each cell
        for (wall_index = 0; wall_index < 4; wall_index++) {
            //printf("\t@@(%d,%d) %d\n",nearby_cells[cell_index].coords[0], nearby_cells[cell_index].coords[1],wall_index);

            struct Wall *wall = nearby_cells[cell_index].walls[wall_index];

            int wall_not_in_weighted_walls = 1;
            int i;
            for (i = 0; i < currently_weighted_walls; i++) {
                if (wall == weighted_walls[i]) {
                    wall_not_in_weighted_walls = 0;
                    break;
                }
            }
            if (wall_not_in_weighted_walls &&
                intersects(wall->line[0], wall->line[1], sensor_pos_in_eevee, sensor_positions)) {
                int dist = dist_to_line_segment(sensor_pos_in_eevee, wall->line[0], wall->line[1]);
                //printf("Checking line %d %d and %d %d\n",wall->line[0][0], wall->line[0][1], wall->line[1][0], wall->line[1][1]);
                if (dist < cell_width) {
                    double trust_val = trust_based_on_distance(dist);
                    printf("Clearin cell (%d,%d) and wall %d with -%f\n", nearby_cells[cell_index].index[0],
                           nearby_cells[cell_index].index[1], wall_index, trust_val);
                    weigh_wall(wall, -trust_val);
                    weighted_walls[currently_weighted_walls] = wall;
                    currently_weighted_walls++;
                }
            }
        }
    }*/

}

int intersects(int *AB, int *CD, int *PQ, int *RS) {
    float det = (CD[0] - AB[0]) * (RS[1] - PQ[1]) - (RS[0] - PQ[0]) * (CD[1] - AB[1]);
    if (det == .0) return 0;

    float lambda_ = ((RS[1] - PQ[1]) * (RS[0] - AB[0]) + (PQ[0] - RS[0]) * (RS[1] - AB[1])) / det;
    float gamma = ((AB[1] - CD[1]) * (RS[0] - AB[0]) + (CD[0] - AB[0]) * (RS[1] - AB[1])) / det;

    return .0 < lambda_ && lambda_ < 1. && 0. < gamma && gamma < 1.;
}


double trust_based_on_distance(int dist) {
    dist = min(cell_width, max(dist, 0));
    return (10.0 * (cell_width - dist)) / cell_width; // ratio proportional to weight [10 ; 0]
}


/*Functions of Wall*/
void weigh_wall(struct Wall *w, double val) {
    if (!is_confirmed_no_wall(w)) {
        w->weight = w->weight + val;

        if (w->weight < min_val) w->weight = min_val;
        if (w->weight > max_val) w->weight = max_val;
        //is_wall
        if (w->weight > 50)
            w->wall |= wall_mask; //1
        else
            w->wall &= ~wall_mask; //0
        //is_no_wall
        if (w->weight < -50)
            w->wall |= no_wall_mask; //1
        else
            w->wall &= ~no_wall_mask; //0

    }
}


void confirm_no_wall(struct Wall *w) {
    w->wall |= (no_wall_conf_mask | no_wall_mask);
    w->wall &= (no_wall_conf_mask | no_wall_mask);
}

void confirm_wall(struct Wall *w) {
    //printf("confirming %x\n",w);
    w->wall |= (wall_conf_mask | wall_mask);
    w->wall &= (wall_conf_mask | wall_mask);
}

int is_confirmed_no_wall(struct Wall *w) {
    return (w->wall & no_wall_conf_mask) != 0;
}

int is_no_wall(struct Wall *w) {
    return (w->wall & no_wall_mask) != 0;
}

int is_wall(struct Wall *w) {
    return (w->wall & wall_mask) != 0;
}


struct Line {
    int a[2];
    int b[2];
};

struct Line beaconLines[10];

int beaconLines_size = 0;
int index_beacon = 0;
int beaconPoints[10][2];
int beaconPoints_size = 0;
int beaconPoints_index = 0;

// add a new beacon line, mark a new beacon point and return avg beacon point
//dir degrees
bool getDirectionTarget(int *curr, double dir, int *beaconPoint) {
    dir = dir * PI / 180;

    // check if we have any beacon line pointing at the beacon and starting close to current position
    int i;
    for (i = 0; i < beaconLines_size; ++i) {
        if (dist(beaconLines[i].a, curr) < 4500) {
            /*printf("current pos too close to another beacon line, %d,%d to %d,%d\n",
                   beaconLines[i].a[0],beaconLines[i].a[1],curr[0], curr[1]);
            setVel2(0,0);
            while(!stopButton());*/

            return false;
        }
    }

    // compute line
    int dist = 60000;
    int newX, newY;

    newX = (int) (curr[0] + cos(dir) * dist);
    newY = (int) (curr[1] - sin(dir) * dist);

    int endLine[2] = {newX, newY};

    // if we have no such line, add a new line and mark the point it intersects with other lines
    struct Line newLine = {.a[0] = curr[0], .a[1]=curr[1], .b[0] = endLine[0], .b[1] = endLine[1]};
    beaconLines[index_beacon] = newLine;
    beaconLines_size = (beaconLines_size >= 10) ? 10 : beaconLines_size + 1;
    printf("new beacon line, %d,%d to %d,%d\n", curr[0], curr[1], endLine[0], endLine[1]);

    /*if we have more than one line in buffer try to intersect them*/
    if (beaconLines_size > 1) {
        int x3 = newLine.a[0];
        int y3 = newLine.a[1];
        int x4 = newLine.b[0];
        int y4 = newLine.b[1];

        for (i = 0; i < beaconLines_size; ++i) {
            if (i == index_beacon)  // skip self
                continue;

            int x1 = beaconLines[i].a[0];
            int y1 = beaconLines[i].a[1];
            int x2 = beaconLines[i].b[0];
            int y2 = beaconLines[i].b[1];

            //if parallel with other line, skip it
            if ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) == 0) {
                continue;
            }

            printf("intersecting a(%d,%d) b(%d,%d) with a(%d,%d) b(%d,%d)\n",x1,y1,x2,y2,x3,y3,x4,y4);
            //intersect with other line
            double Px = 1.0*((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))
                     / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            double Py = 1.0*((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))
                     / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

            //if point between both lines' end points, keep it
            if (Px > min(x3, x4) && Px < max(x3, x4) && Py > min(y3, y4) && Py < max(y3, y4)) {
                setUniqueBeaconPoint((int)Px, (int)Py);
            }
        }
    }
    index_beacon = (index_beacon + 1) % 10;

    return getBeaconAvgPoint(beaconPoint);
}

//*bp =  (int *)malloc(2*(sizeof(int)));

bool getBeaconAvgPoint(int *bp) {
    if (beaconPoints_size <= 0)
        return false;

    int y_total = 0, x_total = 0;
    int i;
    for (i = 0; i < beaconPoints_size; ++i) {
        x_total = x_total + beaconPoints[i][0];
        y_total = y_total + beaconPoints[i][1];
    }
    bp[0] = x_total / beaconPoints_size;
    bp[1] = y_total / beaconPoints_size;

    printf("\tAvg beacon point! (%d, %d)\n",bp[0],bp[1]);
    setVel2(0,0);
    while(!stopButton());

    return true;
}


bool setUniqueBeaconPoint(int x, int y) {
    if (x < -36000 || y < -36000 || x >= 36000 || y >= 36000) {
        return false;
    }
    int i;
    for (i = 0; i < beaconPoints_size; ++i) {
        if (beaconPoints[i][0] == x && beaconPoints[i][1] == y) {
            return true;
        }
    }

    printf("\tAdding unique beacon point! (%d, %d)\n",x,y);
    setVel2(0,0);
    while(!stopButton());
    beaconPoints[beaconPoints_index % 10][0] = x;
    beaconPoints[beaconPoints_index % 10][1] = y;
    beaconPoints_index = (beaconPoints_index + 1) % 10;
    beaconPoints_size = (beaconPoints_size >= 10) ? 10 : beaconPoints_size + 1;


    return true;
}
