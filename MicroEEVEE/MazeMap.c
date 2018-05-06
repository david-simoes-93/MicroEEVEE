#include "MazeMap.h"
#include <math>
/*Functions of Maze*/
#define min(X, Y) (((X) <= (Y)) ? (X) : (Y))
#define max(X,Y) (((X) >= (Y)) ? (X) : (Y))

/*init map*/
void init_maze() {

    /*TODO cell.coord*/
    int i, j;
    for (i = 0; i < cols; i++) {
        for (j = 0; j < rows; j++) {
            float x = (float) i;
            float y = (float) j;
            if (j > 0) {
                /*go find the adjacent wall*/
                maze[j][i].north_wall = maze[j - 1][i].south_wall;
            } else if (j == rows - 1) {
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y + 0.4375},
                            {x + 0.4375, y + 0.4375}}; //line coords
                maze[j][i].south_wall = &tmp;
            } else {
                /*create a new one*/
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y - 0.4375},
                            {x + 0.4375, y - 0.4375}}; //line coords
                maze[j][i].north_wall = &tmp;
            }

            if (i > 0) {
                /*go find the adjacent wall*/
                maze[j][i].west_wall = maze[j][i - 1].east_wall;
            } else if (i == cols - 1) {
                /*last wall on te map without adjacent wall*/
                struct Wall tmp;
                tmp.line = {{x + 0.4375, y - 0.4375},
                            {x + 0.4375, y + 0.4375}};
                maze[j][i].east_wall = &tmp;
            } else {
                /*create a new one*/
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y - 0.4375},
                            {x - 0.4375, y + 0.4375}};
                maze[j][i].west_wall = &tmp;
            }

        }


    }
}

void print_map(){
    for(int x=0; x<cols; x++){
        for(int y=0; y<rows; y++){
            if((maze[x][y].west_wall->wall & wall_mask) != 0)
                printf("|");
            else
                printf(" ");

            if((maze[x][y].south_wall->wall & wall_mask) != 0)
                printf("_");
            else
                printf(" ");
        }
        printf("\n");
    }
}

void get_cell_coords_from_gps_coords(float x, float y, int* tmp){
    
    int t[2];
    t[0] = (int)x/45 + 8;
    t[1] = (int)y/45 + 8;
    tmp = t;

}

// returns dot product of "v" and "w"
float dist2(float* v, float* w){
    return (v[0] - w[0])*(v[0] - w[0]) + (v[1] - w[1]) *(v[1] - w[1]);
}
    

// returns euclidian distance between "v" and "w"
float dist(float* v,float* w){
    return sqrt(dist2(v, w));
}


// returns the square of the distance of point "p" to line segment between "v" and "w"
float dist_to_segment_squared(float* p, float v, float w){
    float l2 = dist2(v, w);
    if (l2 == 0) { return dist2(p, v) ;}
        

    float t;
    t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2 ;
    t = max([0, min([1, t])]);

    return dist2(p, [v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])]);
}

// returns distance of point "p" to line segment between "v" and "w"
float dist_to_line_segment(float* p, float v,  float w){
    return sqrt(dist_to_segment_squared( p, v, w));
}
    


/*ignora os valores acima de 60 cm: demasiado ruido*/
// compass in RADIANS
void update_map(float my_x, float my_y, int left_sensor, int front_sensor, int right_sensor, float compass) {
    float sensor_cutoff_point = 0; //TODO
    float my_pos[2] = {my_x, my_y};

    int *my_cell_index;
    get_cell_coords_from_gps_coords(my_x, my_y,my_cell_index );
    struct Cell my_cell = maze[my_cell_index[0]][my_cell_index[1]];
    my_cell.explored = 1;

    struct Cell nearby_cells[9] = {my_cell,
                                   maze[my_cell_index[0] + 1][my_cell_index[1]],
                                   maze[my_cell_index[0] - 1][my_cell_index[1]],
                                   maze[my_cell_index[0]][my_cell_index[1] + 1],
                                   maze[my_cell_index[0]][my_cell_index[1] - 1],
                                   maze[my_cell_index[0] + 1][my_cell_index[1] + 1],
                                   maze[my_cell_index[0] - 1][my_cell_index[1] - 1],
                                   maze[my_cell_index[0] - 1][my_cell_index[1] + 1],
                                   maze[my_cell_index[0] + 1][my_cell_index[1] - 1]};
    float min_left_sensor = sensor_cutoff_point, min_front_sensor = sensor_cutoff_point, min_right_sensor = sensor_cutoff_point;
    if (left_sensor < sensor_cutoff_point) min_left_sensor = left_sensor;
    if (front_sensor < sensor_cutoff_point) min_front_sensor = front_sensor;
    if (right_sensor < sensor_cutoff_point) min_right_sensor = right_sensor;

    // save points for front of sensor and 30º to left and right of each sensor (they're cone sensors)
    float front_sensor_dots[3][2];
    front_sensor_dots[0][0] = my_x + min_front_sensor / 2 * cos(compass);
    front_sensor_dots[0][1] = my_y + min_front_sensor / 2 * sin(compass);
    front_sensor_dots[1][0] = my_x + min_front_sensor / 2 * cos(compass + pi_over_6);
    front_sensor_dots[1][1] = my_y + min_front_sensor / 2 * sin(compass + pi_over_6);
    front_sensor_dots[2][0] = my_x + min_front_sensor / 2 * cos(compass - pi_over_6);
    front_sensor_dots[2][1] = my_y + min_front_sensor / 2 * sin(compass - pi_over_6);
    update_single_sensor(front_sensor, nearby_cells, front_sensor_dots, my_pos);

    float left_sensor_dots[3][2];
    left_sensor_dots[0][0] = my_x + min_left_sensor / 2 * cos(compass - pi_over_2);
    left_sensor_dots[0][1] = my_y + min_left_sensor / 2 * sin(compass - pi_over_2);
    left_sensor_dots[1][0] = my_x + min_left_sensor / 2 * cos(compass - pi_over_2 + pi_over_6);
    left_sensor_dots[1][1] = my_y + min_left_sensor / 2 * sin(compass - pi_over_2 + pi_over_6);
    left_sensor_dots[2][0] = my_x + min_left_sensor / 2 * cos(compass - pi_over_2 - pi_over_6);
    left_sensor_dots[2][1] = my_y + min_left_sensor / 2 * sin(compass - pi_over_2 - pi_over_6);
    update_single_sensor(left_sensor, nearby_cells, left_sensor_dots, my_pos);

    float right_sensor_dots[3][2];
    right_sensor_dots[0][0] = my_x + min_right_sensor / 2 * cos(compass + pi_over_2);
    right_sensor_dots[0][1] = my_y + min_right_sensor / 2 * sin(compass + pi_over_2);
    right_sensor_dots[1][0] = my_x + min_right_sensor / 2 * cos(compass + pi_over_2 + pi_over_6);
    right_sensor_dots[1][1] = my_y + min_right_sensor / 2 * sin(compass + pi_over_2 + pi_over_6);
    right_sensor_dots[2][0] = my_x + min_right_sensor / 2 * cos(compass + pi_over_2 - pi_over_6);
    right_sensor_dots[2][1] = my_y + min_right_sensor / 2 * sin(compass + pi_over_2 - pi_over_6);
    update_single_sensor(right_sensor, nearby_cells, right_sensor_dots, my_pos);

    // confirm no walls where we are moving through
    // run through all neighbor cells
    for (int cell_index = 0; cell_index < 9; cell_index++) {
        // and 4 walls on each cell
        for (int wall_index = 0; wall_index < 4; wall_index++) {
            struct Wall wall = nearby_cells[cell_index].walls[wall_index];
            if (dist_to_line_segment(my_pos, wall.line[0], wall.line[1]) < 9) wall.confirm_no_wall();
        }
    }
}

/*adiciona o valor*/
void update_single_sensor(float sensor_val, struct Cell *nearby_cells, float **sensor_positions,
                          float *sensor_pos_in_eevee) 
{
    float sensor_cutoff_point = 0; //TODO
    float max_dist_threshold = 0; //TODO

    // if obstacle found
    int possible_walls_size = 3 * 9 * 4;
    int currently_weighted_walls = 0;
    struct Wall *weighted_walls[possible_walls_size];
    if (sensor_val < sensor_cutoff_point)  // self.max_dist_threshold:
    {
        // check closest ray to any wall
        float ray_dists[3] = {max_dist_threshold, max_dist_threshold, max_dist_threshold};
        struct Wall *ray_walls[3];

        // check 30º to the left, 0º, and 30º to the right; 3 possible sensor points
        for (int sensor_index = 0; sensor_index < 3; sensor_index++) {
            // run through all neighbor cells
            for (int cell_index = 0; cell_index < 9; cell_index++) {
                // and 4 walls on each cell
                for (int wall_index = 0; wall_index < 4; wall_index++) {
                    struct Wall wall = nearby_cells[cell_index].walls[wall_index];

                    // calculate distance of sensor point to the wall and save closest wall
                    float d = dist_to_line_segment(sensor_positions[sensor_index], wall.line[0], wall.line[1]);
                    if (d < ray_dists[sensor_index]) {
                        ray_dists[sensor_index] = d;
                        ray_walls[sensor_index] = &wall;
                    }
                }
            }
        }

        // store closest possible wall
        int min_index = 0;
        if (ray_dists[1] < ray_dists[0] && ray_dists[1] < ray_dists[2])
            min_index = 1;
        else if (ray_dists[2] < ray_dists[0] && ray_dists[2] < ray_dists[1])
            min_index = 2;


        // if that ray is actually close to a wall
        // closest_dot_to_wall_distance = ray_dists[min_index]
        float *closest_dot_to_wall = sensor_positions[min_index];
        struct Wall closest_dot_cell_wall = *ray_walls[min_index];

        float trust_val = trust_based_on_distance(dist_to_line_segment(
                sensor_pos_in_eevee, closest_dot_cell_wall.line[0], closest_dot_cell_wall.line[1]));
        closest_dot_cell_wall.weigh(trust_val);
        weighted_walls[0] = &closest_dot_cell_wall;
        currently_weighted_walls++;
    }

    // decrease all other wall intersections score
    // check 30º to the left, 0º, and 30º to the right; 3 possible sensor points
    for (int sensor_index = 0; sensor_index < 3; sensor_index++) {
        // run through all neighbor cells
        for (int cell_index = 0; cell_index < 9; cell_index++) {
            // and 4 walls on each cell
            for (int wall_index = 0; wall_index < 4; wall_index++) {
                struct Wall *wall = &nearby_cells[cell_index].walls[wall_index];

                int wall_not_in_weighted_walls = 1;
                for (int i = 0; i < currently_weighted_walls; i++) {
                    if (wall == weighted_walls[i]) {
                        wall_not_in_weighted_walls = 0;
                        break;
                    }
                }
                if (wall_not_in_weighted_walls &&
                    intersects(wall->line[0], wall->line[1], sensor_pos_in_eevee, sensor_positions[sensor_index])) {
                    float dist = dist_to_line_segment(sensor_pos_in_eevee, wall->line[0], wall->line[1]);
                    if (dist < 1) {
                        float trust_val = trust_based_on_distance(dist);
                        wall->weigh(-trust_val);
                        weighted_walls[currently_weighted_walls] = wall;
                        currently_weighted_walls++;
                    }
                }
            }
        }
    }
}

/*mapeia o peso de confiança para dar a parede*/
int map_weight(float dist) {
    float output_start = -255;
    float output_end = 255;
    float input_start = 45; //cm
    float input_end = 7;
    return (int) (output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start));
}


/*index of wall according to heading*/
int wall_index(float heading);


/*Functions of Wall*/
void weight(struct Wall *w, int val) {
    if (!confirmed_no_wall(w)) {
        w.weight = w.weight + val;
        if (w.weight < min_val) w.weight = min_val;
        if (w.weight > max_val) w.weight = max_val;
        //wall
        if (w.weight > 32) w.wall = w.wall | 0x01; //True
        else w.wall = w.wall & 0xFE; //False
        //no_wall
        if (w.weight < -32) w.wall = w.wall | 0x02; //True
        else w.wall = w.wall & 0xFD; //False

    }
}


void confirm_no_wall(struct Wall *w) {
    w.wall = w.wall | 0x07; // confirmed_no_wall , no_wall = TRue
    w.wall = w.wall & 0x01; //wall = False
}

bool confirmed_no_wall(struct Wall *w) {
    return (w.wall & 0x04) >> 2 == 1 ? true : false;
}

bool no_wall(struct Wall *w) {
    return (w.wall & 0x02) >> 1 == 1 ? true : false;
}

bool wall(struct Wall *w) {
    return (w.wall & 0x01) == 1 ? true : false;
}



struct Line {
    float *a;
    float *b;
}

struct Line beaconLines[10];

int count_beacon_line;
count_beacon_line = 0;
int index_beacon = 0;
// added a new beacon line, mark a new beacon point, try and find a path there
int* getDirectionTarget(float* curr, float dir, Cell actualCurr) {
    bool contains = false;
    // check if we have any beacon line pointing at the beacon and starting close to current position
    for (int i = 0; i < count_beacon_line; ++i)
    {

        if ( dist(beaconLines[i].a, curr) < 3) {
            contains = true;
            break;
        }
    }

    // compute line
    int dist = 5 * 3;
    float newX, newY;
    newX = (curr[0] + cos(dir) * dist);
    newY = (curr[1] - sin(dir) * dist);
    float endLine[2] = {newX, newY };

    // if we have no such line, add a new line and mark the point it intersects with other lines
    if (!contains) {
        struct Line newLine = {.a = curr, .b = &endLine };
        beaconLines[index_beacon%10] = newLine;
        index_beacon = (index_beacon+1) %10;
        count_beacon_line = (count_beacon_line >= 10) ? count_beacon_line : count_beacon_line+1 ;

        float x3 = newLine.a[0];
        float y3 = newLine.a[1];
        float x4 = newLine.b[0];
        float y4 = newLine.b[1];

        for (   int i = 0;  i < count_beacon_line - 1; i++) {
            float x1 = beaconLines[i].a[0];
            float y1 = beaconLines[i].a[1];
            float x2 = beaconLines[i].b[0];
            float y2 = beaconLines[i].b[1];

            //if parallel with other line, skip it
            if ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) == 0) {
                continue;
            }

            //intersect with other line
            float Px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))
                       / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            float Py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))
                       / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

            //if point between both lines' end points, keep it
            if (Px > min(x3, x4) && Px < max(x3, x4) && Py > min(y3, y4) && Py < max(y3, y4 )) {
                // if (gui != null) {
                //     gui.paintPoint((int) Px, (int) Py, Color.RED);
                // }
                
                setUniqueBeaconPoint((  int) Px, (int) Py);
            }
        }
    }

    // if (!beaconPoints.isEmpty() ) {
    //     return  getWayToBeacon(actualCurr);
    // } 
    // else {
    //     Cell tar = new  Cell(newX, newY );
    //     tar = findClosestFreeCell(tar, 2);
    //     return  StarSearchExplorerZone(actualCurr, tar, 2, 3);
    // }
}