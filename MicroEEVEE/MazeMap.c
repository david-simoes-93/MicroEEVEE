#include "MazeMap.h"
#include <math>
/*Functions of Maze*/

/*init map*/
void init() {

    /*TODO cell.coord*/
    int i, j;
    for (i = 0; i < cols; i++) {
        for (j = 0; j < rows; j++) {
            float x = (float) i;
            float y = (float) j;
            if (j > 0) {
                /*go find the adjacent wall*/
                maze[j][i].
                north = maze[j - 1][i].
                south;
            } else if (j == rows - 1) {
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y + 0.4375},
                            {x + 0.4375, y + 0.4375}}; //line coords
                maze[j][i].
                south = &tmp;
            } else {
                /*create a new one*/
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y - 0.4375},
                            {x + 0.4375, y - 0.4375}}; //line coords
                maze[j][i].
                north = &tmp;
            }

            if (i > 0) {
                /*go find the adjacent wall*/
                maze[j][i].
                west = maze[j][i - 1].
                east;
            } else if (i == cols - 1) {
                /*last wall on te map without adjacent wall*/
                struct Wall tmp;
                tmp.line = {{x + 0.4375, y - 0.4375},
                            {x + 0.4375, y + 0.4375}}
                maze[j][i].
                east = &tmp;
            } else {
                /*create a new one*/
                struct Wall tmp;
                tmp.line = {{x - 0.4375, y - 0.4375},
                            {x - 0.4375, y + 0.4375}}
                maze[j][i].
                west = &tmp;
            }

        }


    }
}

/*ignora os valores acima de 60 cm: demasiado ruido*/
// compass in RADIANS
void update_map(float my_x, float my_y, float left_sensor, float front_sensor, float right_sensor, float compass) {
    float sensor_cutoff_point = 0; //TODO
    float my_pos[2] = {my_x, my_y};

    int *my_cell_index = get_cell_coords_from_gps_coords(my_x, my_y);
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
                          float *sensor_pos_in_eevee) {
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


//##########################################################################################################
//############################# JAVA ############################

// added a new beacon line, mark a new beacon point, try and find a path there
public Cell
getDirectionTarget(Cell
curr,
float dir, Cell
actualCurr) {
boolean contains = false;
// check if we have any beacon line pointing at the beacon and starting close to current position
for (
Line l :
beaconLines) {
if (
getDistanceBetweenCells(l
.a, curr) < 3) {
contains = true;
break;
}
}

int dist = 5 * 3;
int newX = (int) (curr.x + cos(dir) * dist);
int newY = (int) (curr.y - sin(dir) * dist);
Cell endLine = new
Cell(newX, newY
);

// if we have no such line, add a new line and mark the point it intersects with other lines
if (!contains) {
Line newLine = new
Line(curr, endLine
);
beaconLines.
add(new
Line(curr, endLine
));

float x3 = newLine.a.x;
float y3 = newLine.a.y;
float x4 = newLine.b.x;
float y4 = newLine.b.y;

for (
int i = 0;
i < beaconLines.

size()

- 1; i++) {
float x1 = beaconLines.get(i).a.x;
float y1 = beaconLines.get(i).a.y;
float x2 = beaconLines.get(i).b.x;
float y2 = beaconLines.get(i).b.y;

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
if (Px > Math.
min(x3, x4
) && Px < Math.
max(x3, x4
) && Py > Math.
min(y3, y4
) && Py < Math.
max(y3, y4
)) {
if (gui != null) {
gui.paintPoint((
int) Px, (int) Py, Color.RED);
}
setUniqueBeaconPoint((
int) Px, (int) Py);
}
}
}

if (!beaconPoints.

isEmpty()

) {
return
getWayToBeacon(actualCurr);
} else {
Cell tar = new
Cell(newX, newY
);
tar = findClosestFreeCell(tar, 2);
return
StarSearchExplorerZone(actualCurr, tar,
2, 3);
}
}