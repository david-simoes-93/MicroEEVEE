#include "MazeMap.h"
#include <math>
/*Functions of Maze*/

/*init map*/
void init(){

	/*TODO cell.coord*/
	int i,j;
	for (i = 0; i < cols; i++)
	{
		for (j = 0; j < rows; j++)
		{
			float x = (float)i;
			float y = (float)j;
			if(j>0){
				/*go find the adjacent wall*/
				maze[j][i].north = maze[j-1][i].south ;
			}
			else if(j== rows-1){
				struct Wall tmp;
				tmp.line = {{x - 0.4375, y + 0.4375}, {x + 0.4375, y + 0.4375}}; //line coords
				maze[j][i].south = &tmp;
			}
			else{
				/*create a new one*/
				struct Wall tmp;
				tmp.line = {{x - 0.4375, y - 0.4375}, {x + 0.4375, y - 0.4375}}; //line coords
				maze[j][i].north = &tmp;
			}

			if(i>0){
				/*go find the adjacent wall*/
				maze[j][i].west = maze[j][i-1].east ;
			}
			else if(i== cols-1){
				/*last wall on te map without adjacent wall*/
				struct Wall tmp;
				tmp.line = {{x + 0.4375, y - 0.4375}, {x + 0.4375, y + 0.4375}}
				maze[j][i].east = &tmp;
			}
			else{
				/*create a new one*/
				struct Wall tmp;
				tmp.line = {{x - 0.4375, y - 0.4375}, {x - 0.4375, y + 0.4375}}
				maze[j][i].west = &tmp;
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

/*ignora os valores acima de 60 cm: demasiado ruido*/
void update_map(double x, double y, double left, double front, double right ,double compass, int ground );

/*adiciona o valor*/
void update_single_sensor(double sensor_val, struct Cell[8], double sensor_positions[3][2] ,  double sensor_pos_in_eevee[2]){
	// if obstacle found
    weighted_walls = []
    if sensor_val < self.sensor_cutoff_point:  // self.max_dist_threshold:
        # check closest ray to any wall
        ray_dists = [self.max_dist_threshold, self.max_dist_threshold, self.max_dist_threshold]
        ray_walls = [None, None, None]
        for index, dot in enumerate(sensor_positions):
            for cell in nearby_cells:
                for wall in cell.walls:
                    d = dist_to_line_segment(dot, wall.line[0], wall.line[1])
                    if d < ray_dists[index]:
                        ray_dists[index] = d
                        ray_walls[index] = wall
        min_index = np.argmin(ray_dists)

        /* if that ray is actually close to a wall
        # closest_dot_to_wall_distance = ray_dists[min_index]
        # if closest_dot_to_wall_distance < .1:  # TODO account of noise increase as dist increases - sensor_val / 10 */
        closest_dot_to_wall = sensor_positions[min_index]
        closest_dot_cell_wall = ray_walls[min_index]
        self.wall_dots.append(closest_dot_to_wall)
        trust_val = self.trust_based_on_distance(
            dist_to_line_segment(sensor_pos_in_eevee, closest_dot_cell_wall.line[0], closest_dot_cell_wall.line[1]))
        // print("found obs", closest_dot_cell_wall, trust_val)
        closest_dot_cell_wall.weigh(trust_val)
        closest_dot_cell_wall.get_adjacent_wall().weigh(trust_val)
        weighted_walls = [closest_dot_cell_wall]
        // print("wall", closest_dot_cell_wall)

    // decrease all other wall intersections score
    for dot in sensor_positions:
        for cell in nearby_cells:
            for wall in cell.walls:
                if wall not in weighted_walls and intersects(wall.line[0], wall.line[1],
                                                             sensor_pos_in_eevee, dot):
                    dist = dist_to_line_segment(sensor_pos_in_eevee, wall.line[0], wall.line[1])
                    # if dist < 1:
                    trust_val = self.trust_based_on_distance(dist - 0.25)
                    wall.weigh(-trust_val)
                    wall.get_adjacent_wall().weigh(-trust_val)
                    # print("cleared wall", wall, trust_val)
                    # print("no wall", wall)
                    weighted_walls.append(wall)
        self.sensor_dots.append(dot)

}

/*mapeia o peso de confiança para dar a parede*/
int map_weight(double dist){
	double output_start = -255;
	double output_end = 255;
	double input_start = 45; //cm
	double input_end = 7;
	return (int)(output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start));
}


/*index of wall according to heading*/
int wall_index(double heading);


/*Functions of Wall*/
void weight(struct Wall* w, int val){
	if( !confirmed_no_wall(w) ){
		w.weight = w.weight + val;
		if (w.weight < min_val ) w.weight = min_val;
		if (w.weight > max_val ) w.weight = max_val;
		//wall
		if (w.weight > 32) w.wall = w.wall | 0x01 ; //True
		else w.wall = w.wall & 0xFE; //False
		//no_wall
		if (w.weight < -32) w.wall = w.wall | 0x02 ; //True
		else w.wall = w.wall & 0xFD; //False

	}
}


void confirm_no_wall(struct Wall* w){
    w.wall = w.wall | 0x07 ; // confirmed_no_wall , no_wall = TRue
    w.wall = w.wall & 0x01; //wall = False
}

bool confirmed_no_wall(struct Wall* w){
	return (w.wall & 0x04)>>2 == 1 ? true: false; 
}
bool no_wall(struct Wall* w){
	return (w.wall & 0x02)>>1 == 1 ? true: false; 
}
bool wall(struct Wall* w){
	return (w.wall & 0x01) == 1 ? true: false; 
}

