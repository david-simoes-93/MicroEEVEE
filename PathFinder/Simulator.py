import random
import time
import math
import copy

import OdometryHandler
import Utils
from Utils import Location


class Cell:
    def __init__(self, up, down, left, right):
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.goal = False

    def set(self, up, down, left, right):
        self.up = up
        self.down = down
        self.left = left
        self.right = right


MAP_SIZE = 20
CM_PER_CELL = 12.5
MAX_SPEED_CM_PER_SEC = 10
MOTOR_NOISE_RATIO =  0.1
SENSOR_NOISE_RATIO = 0.02


class MazeSimulator:
    def __init__(self):
        # linhas pretas de 2.5cm
        # área total de 240cmx240cm.
        # A distância mínima entre o eixo central de duas linhas paralelas é 12.5 cm
        # o espaço mínimo entre os centros de duas interseções ou entre o centro de uma interseção e uma mudança de direção é 25 cm
        # O ponto de chegada do labirinto é marcado com um retângulo preto com 12.5x5.5 cm.
        # Essa marca tem uma distância mínima à linha mais próxima de 5 cm e o segmento de reta que a ela conduz tem um comprimento mínimo de 20cm.

        self.maze = [[Cell(False, False, False, False)
                      for y in range(MAP_SIZE)] for x in range(MAP_SIZE)]
        self.starting_pos = Location(4, 0)
        self.set_map()

        self.eevee_coords = copy.copy(Location(4.2, 0))
        self.eevee_theta = 0

        self.last_update = time.time()

        self.l_pwm = 0
        self.r_pwm = 0

        self.encLeft = 0
        self.encRight = 0

    def set_motor_pwm(self, l_pwm, r_pwm):
        if self.l_pwm == l_pwm and self.r_pwm == r_pwm:
            return
        #self.update_loc()
        # [-90, 90] Hz
        self.l_pwm = l_pwm
        self.r_pwm = r_pwm
        
        # TODO: momentum is lost, but shouldn't be

    def get_ground(self):
        # 40º left
        far_left_sensor_coords = Location(self.eevee_coords.x + Utils.GROUND_SENSOR_DISTANCE * math.cos(self.eevee_theta - Utils.FAR_SENSOR_ANGLE) / CM_PER_CELL,
                                  self.eevee_coords.y + Utils.GROUND_SENSOR_DISTANCE * math.sin(self.eevee_theta - Utils.FAR_SENSOR_ANGLE) / CM_PER_CELL)
        # 10º left
        left_sensor_coords = Location(self.eevee_coords.x + Utils.GROUND_SENSOR_DISTANCE * math.cos(self.eevee_theta - Utils.NEAR_SENSOR_ANGLE) / CM_PER_CELL,
                              self.eevee_coords.y + Utils.GROUND_SENSOR_DISTANCE * math.sin(self.eevee_theta - Utils.NEAR_SENSOR_ANGLE) / CM_PER_CELL)
        # 0º front
        front_sensor_coords = Location(self.eevee_coords.x + Utils.GROUND_SENSOR_DISTANCE * math.cos(self.eevee_theta) / CM_PER_CELL,
                               self.eevee_coords.y + Utils.GROUND_SENSOR_DISTANCE * math.sin(self.eevee_theta) / CM_PER_CELL)
        # 10º right
        right_sensor_coords = Location(self.eevee_coords.x + Utils.GROUND_SENSOR_DISTANCE * math.cos(self.eevee_theta + Utils.NEAR_SENSOR_ANGLE) / CM_PER_CELL,
                               self.eevee_coords.y + Utils.GROUND_SENSOR_DISTANCE * math.sin(self.eevee_theta + Utils.NEAR_SENSOR_ANGLE) / CM_PER_CELL)
        # 40º right
        far_right_sensor_coords = Location(self.eevee_coords.x + Utils.GROUND_SENSOR_DISTANCE * math.cos(self.eevee_theta + Utils.FAR_SENSOR_ANGLE) / CM_PER_CELL,
                                   self.eevee_coords.y + Utils.GROUND_SENSOR_DISTANCE * math.sin(self.eevee_theta + Utils.FAR_SENSOR_ANGLE) / CM_PER_CELL)
        
        sensor_coords = [far_left_sensor_coords, left_sensor_coords, front_sensor_coords, right_sensor_coords, far_right_sensor_coords] # [ [1.1, 0.9], ... ]
        sensor_cell_indices = [Location(round(coords.x), round(coords.y)) for coords in sensor_coords]                                  # [   [1, 1],   ... ]
        rel_sensor_coords = [coords-cell for coords,cell in zip(sensor_coords, sensor_cell_indices)]                                    # [ [0.1,-0.1], ... ]
        sensor_cells = [self.maze[cell_index.x][cell_index.y] for cell_index in sensor_cell_indices]                                    # [  Cell_1_1 , ... ]
        sensors = [self.is_sensor_over_a_line(cell, rel_coords) for cell, rel_coords in zip(sensor_cells, rel_sensor_coords)]           # [    1/0    , ... ]
        return sensors
    
    def is_sensor_over_a_line(self, cell, rel_coords):
        if cell.up:
            if -0.1 <= rel_coords.x <= 0.1 and -0.5 <= rel_coords.y <= 0:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        if cell.down:
            if -0.1 <= rel_coords.x <= 0.1 and 0 <= rel_coords.y <= 0.5:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        if cell.left:
            if -0.5 <= rel_coords.x <= 0 and -0.1 <= rel_coords.y <= 0.1:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        if cell.right:
            if 0 <= rel_coords.x <= 0.5 and -0.1 <= rel_coords.y <= 0.1:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        if cell.goal and (cell.right or cell.left):
            if -0.22 <= rel_coords.x <= 0.22 and -0.5 <= rel_coords.y <= 0.5:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        if cell.goal and (cell.up or cell.down):
            if -0.5 <= rel_coords.x <= 0.5 and -0.22 <= rel_coords.y <= 0.22:
                return True if random.random() > SENSOR_NOISE_RATIO else False
        return False if random.random() > SENSOR_NOISE_RATIO else True

    def get_encoder(self):
        encoders = [self.encLeft, self.encRight]
        self.encLeft = 0
        self.encRight = 0
        return encoders
    
    def update_loc(self):
        curr_time = time.time()
        time_elapsed = curr_time - self.last_update
        dLeft = MAX_SPEED_CM_PER_SEC * time_elapsed * (self.l_pwm/100)
        dRight = MAX_SPEED_CM_PER_SEC * time_elapsed * (self.r_pwm/100)
        dLeft *= (1 + random.random() * MOTOR_NOISE_RATIO * 2 - MOTOR_NOISE_RATIO)
        dRight *= (1 + random.random() * MOTOR_NOISE_RATIO * 2 - MOTOR_NOISE_RATIO)
        
        dCenter = (dLeft + dRight) / 2.0
        phi = (dLeft - dRight) / OdometryHandler.WHEEL2WHEEL_DIST

        new_eevee_theta = Utils.normalize_radian_angle(self.eevee_theta + phi)
        avg_theta = Utils.avg_between_radian_angles(self.eevee_theta, new_eevee_theta)
        x_delta_cm = dCenter * math.cos(avg_theta)
        y_delta_cm = dCenter * math.sin(avg_theta)
        self.eevee_coords.x += x_delta_cm/CM_PER_CELL
        self.eevee_coords.y += y_delta_cm/CM_PER_CELL
        self.eevee_theta = new_eevee_theta
        
        self.encLeft += dLeft * OdometryHandler.GEAR_RATIO_times_ENCODER_PULSES / OdometryHandler.WHEEL_PER
        self.encRight += dRight * OdometryHandler.GEAR_RATIO_times_ENCODER_PULSES / OdometryHandler.WHEEL_PER

        self.last_update = curr_time

    def set_map(self):
        # example map from https://drive.google.com/file/d/1_pOQQkb6gatRcMIIKV23zb5OahAf3XqW/view
        self.maze[0][0].set(up=False, down=True, left=False, right=True)
        self.maze[1][0].set(up=False, down=False, left=True, right=True)
        self.maze[2][0].set(up=False, down=False, left=True, right=True)
        self.maze[3][0].set(up=False, down=True, left=True, right=True)
        self.maze[4][0].set(up=False, down=False, left=True, right=True)
        self.maze[5][0].set(up=False, down=True, left=True, right=True)
        self.maze[6][0].set(up=False, down=False, left=True, right=True)
        self.maze[7][0].set(up=False, down=False, left=True, right=False)
        self.maze[8][0].set(up=False, down=True, left=False, right=False)

        self.maze[0][1].set(up=True, down=True,left=False, right=False)
        self.maze[1][1].set(up=False, down=False, left=False, right=False)
        self.maze[2][1].set(up=False, down=False, left=False, right=False)
        self.maze[3][1].set(up=True, down=True, left=False, right=False)
        self.maze[4][1].set(up=False, down=False, left=False, right=False)
        self.maze[5][1].set(up=True, down=True, left=False, right=False)
        self.maze[6][1].set(up=False, down=False, left=False, right=False)
        self.maze[7][1].set(up=False, down=False, left=False, right=False)
        self.maze[8][1].set(up=True, down=True, left=False, right=False)

        self.maze[0][2].set(up=True, down=False, left=False, right=True)
        self.maze[1][2].set(up=False, down=False, left=True, right=True)
        self.maze[2][2].set(up=False, down=True, left=True, right=False)
        self.maze[3][2].set(up=True, down=False, left=False, right=False)
        self.maze[4][2].set(up=False, down=False, left=False, right=False)
        self.maze[5][2].set(up=True, down=True, left=False, right=False)
        self.maze[6][2].set(up=False, down=False, left=False, right=True)
        self.maze[7][2].set(up=False, down=False, left=True, right=True)
        self.maze[8][2].set(up=True, down=True, left=True, right=False)

        self.maze[0][3].set(up=False, down=False, left=False, right=False)
        self.maze[1][3].set(up=False, down=False, left=False, right=False)
        self.maze[2][3].set(up=True, down=True, left=False, right=False)
        self.maze[3][3].set(up=False, down=False, left=False, right=True)
        #self.maze[3][3].goal = True
        self.maze[4][3].set(up=False, down=False, left=True, right=True)
        self.maze[5][3].set(up=True, down=True, left=True, right=True)
        self.maze[6][3].set(up=False, down=False, left=True, right=True)
        self.maze[7][3].set(up=False, down=False, left=True, right=False)
        self.maze[8][3].set(up=True, down=True, left=False, right=False)

        self.maze[0][4].set(up=False, down=True, left=False, right=True)
        self.maze[1][4].set(up=False, down=False, left=True, right=True)
        self.maze[2][4].set(up=True, down=False, left=True, right=True)
        self.maze[3][4].set(up=False, down=False, left=True, right=True)
        self.maze[4][4].set(up=False, down=False, left=True, right=False)
        self.maze[5][4].set(up=True, down=True, left=False, right=False)
        self.maze[6][4].set(up=False, down=False, left=False, right=True)
        self.maze[7][4].set(up=False, down=False, left=True, right=True)
        self.maze[8][4].set(up=True, down=True, left=True, right=True)

        self.maze[0][5].set(up=True, down=True, left=False, right=False)
        self.maze[1][5].set(up=False, down=False, left=False, right=True)
        self.maze[2][5].set(up=False, down=False, left=True, right=True)
        self.maze[3][5].set(up=False, down=True, left=True, right=True)
        self.maze[4][5].set(up=False, down=False, left=True, right=True)
        self.maze[5][5].set(up=True, down=True, left=True, right=True)
        self.maze[6][5].set(up=False, down=False, left=True, right=True)
        self.maze[7][5].set(up=False, down=False, left=True, right=False)
        self.maze[8][5].set(up=True, down=True, left=False, right=False)

        self.maze[0][6].set(up=True, down=True, left=False, right=True)
        self.maze[1][6].set(up=False, down=False, left=True, right=True)
        self.maze[2][6].set(up=False, down=True, left=True, right=False)
        self.maze[3][6].set(up=True, down=True, left=False, right=False)
        self.maze[4][6].set(up=False, down=True, left=False, right=False)
        self.maze[5][6].set(up=True, down=True, left=False, right=False)
        self.maze[6][6].set(up=False, down=False, left=False, right=False)
        self.maze[7][6].set(up=False, down=False, left=False, right=False)
        self.maze[8][6].set(up=True, down=True, left=False, right=False)

        self.maze[0][7].set(up=True, down=True, left=False, right=False)
        self.maze[1][7].set(up=False, down=False, left=False, right=False)
        self.maze[2][7].set(up=True, down=True, left=False, right=False)
        self.maze[3][7].set(up=True, down=False, left=False, right=False)
        self.maze[4][7].set(up=True, down=True, left=False, right=False)
        self.maze[5][7].set(up=True, down=False, left=False, right=True)
        self.maze[6][7].set(up=False, down=False, left=True, right=True)
        self.maze[7][7].set(up=False, down=False, left=True, right=True)
        self.maze[8][7].set(up=True, down=False, left=True, right=False)

        self.maze[0][8].set(up=True, down=False, left=False, right=False)
        self.maze[1][8].set(up=False, down=False, left=False, right=False)
        self.maze[2][8].set(up=True, down=False, left=False, right=True)
        self.maze[3][8].set(up=False, down=False, left=True, right=True)
        self.maze[4][8].set(up=True, down=True, left=True, right=True)
        self.maze[5][8].set(up=False, down=False, left=True, right=True)
        self.maze[6][8].set(up=False, down=False, left=True, right=False)
        self.maze[7][8].set(up=False, down=False, left=False, right=False)
        self.maze[8][8].set(up=False, down=False, left=False, right=False)

        # connections
        self.maze[9][4].set(up=False, down=False, left=True, right=True)
        self.maze[4][9].set(up=True, down=True, left=False, right=False)
        self.maze[14][9].set(up=True, down=True, left=False, right=False)

        # bottom left
        self.maze[0][10].set(up=False, down=True, left=False, right=True)
        self.maze[1][10].set(up=False, down=False, left=True, right=True)
        self.maze[2][10].set(up=False, down=False, left=True, right=True)
        self.maze[3][10].set(up=False, down=True, left=True, right=True)
        self.maze[4][10].set(up=True, down=False, left=True, right=True)
        self.maze[5][10].set(up=False, down=True, left=True, right=True)
        self.maze[6][10].set(up=False, down=False, left=True, right=True)
        self.maze[7][10].set(up=False, down=False, left=True, right=False)
        self.maze[8][10].set(up=False, down=True, left=False, right=False)

        self.maze[0][11].set(up=True, down=True,left=False, right=False)
        self.maze[1][11].set(up=False, down=False, left=False, right=False)
        self.maze[2][11].set(up=False, down=False, left=False, right=False)
        self.maze[3][11].set(up=True, down=True, left=False, right=False)
        self.maze[4][11].set(up=False, down=False, left=False, right=False)
        self.maze[5][11].set(up=True, down=True, left=False, right=False)
        self.maze[6][11].set(up=False, down=False, left=False, right=False)
        self.maze[7][11].set(up=False, down=False, left=False, right=False)
        self.maze[8][11].set(up=True, down=True, left=False, right=False)

        self.maze[0][12].set(up=True, down=False, left=False, right=True)
        self.maze[1][12].set(up=False, down=False, left=True, right=True)
        self.maze[2][12].set(up=False, down=True, left=True, right=False)
        self.maze[3][12].set(up=True, down=False, left=False, right=False)
        self.maze[4][12].set(up=False, down=False, left=False, right=False)
        self.maze[5][12].set(up=True, down=True, left=False, right=False)
        self.maze[6][12].set(up=False, down=False, left=False, right=True)
        self.maze[7][12].set(up=False, down=False, left=True, right=True)
        self.maze[8][12].set(up=True, down=True, left=True, right=False)

        self.maze[0][13].set(up=False, down=False, left=False, right=False)
        self.maze[1][13].set(up=False, down=False, left=False, right=False)
        self.maze[2][13].set(up=True, down=True, left=False, right=False)
        self.maze[3][13].set(up=False, down=False, left=False, right=True)
        self.maze[4][13].set(up=False, down=False, left=True, right=True)
        self.maze[5][13].set(up=True, down=True, left=True, right=True)
        self.maze[6][13].set(up=False, down=False, left=True, right=True)
        self.maze[7][13].set(up=False, down=False, left=True, right=False)
        self.maze[8][13].set(up=True, down=True, left=False, right=False)

        self.maze[0][14].set(up=False, down=True, left=False, right=True)
        self.maze[1][14].set(up=False, down=False, left=True, right=True)
        self.maze[2][14].set(up=True, down=False, left=True, right=True)
        self.maze[3][14].set(up=False, down=False, left=True, right=True)
        self.maze[4][14].set(up=False, down=False, left=True, right=False)
        self.maze[5][14].set(up=True, down=True, left=False, right=False)
        self.maze[6][14].set(up=False, down=False, left=False, right=True)
        self.maze[7][14].set(up=False, down=False, left=True, right=True)
        self.maze[8][14].set(up=True, down=True, left=True, right=False)

        self.maze[0][15].set(up=True, down=True, left=False, right=False)
        self.maze[1][15].set(up=False, down=False, left=False, right=True)
        self.maze[2][15].set(up=False, down=False, left=True, right=True)
        self.maze[3][15].set(up=False, down=True, left=True, right=True)
        self.maze[4][15].set(up=False, down=False, left=True, right=True)
        self.maze[5][15].set(up=True, down=True, left=True, right=True)
        self.maze[6][15].set(up=False, down=False, left=True, right=True)
        self.maze[7][15].set(up=False, down=False, left=True, right=False)
        self.maze[8][15].set(up=True, down=True, left=False, right=False)

        self.maze[0][16].set(up=True, down=True, left=False, right=True)
        self.maze[1][16].set(up=False, down=False, left=True, right=True)
        self.maze[2][16].set(up=False, down=True, left=True, right=False)
        self.maze[3][16].set(up=True, down=True, left=False, right=False)
        self.maze[4][16].set(up=False, down=True, left=False, right=False)
        self.maze[5][16].set(up=True, down=True, left=False, right=False)
        self.maze[6][16].set(up=False, down=False, left=False, right=False)
        self.maze[7][16].set(up=False, down=False, left=False, right=False)
        self.maze[8][16].set(up=True, down=True, left=False, right=False)

        self.maze[0][17].set(up=True, down=True, left=False, right=False)
        self.maze[1][17].set(up=False, down=False, left=False, right=False)
        self.maze[2][17].set(up=True, down=True, left=False, right=False)
        self.maze[3][17].set(up=True, down=False, left=False, right=False)
        self.maze[4][17].set(up=True, down=True, left=False, right=False)
        self.maze[5][17].set(up=True, down=False, left=False, right=True)
        self.maze[6][17].set(up=False, down=False, left=True, right=True)
        self.maze[7][17].set(up=False, down=False, left=True, right=True)
        self.maze[8][17].set(up=True, down=False, left=True, right=False)

        self.maze[0][18].set(up=True, down=False, left=False, right=False)
        self.maze[1][18].set(up=False, down=False, left=False, right=False)
        self.maze[2][18].set(up=True, down=False, left=False, right=True)
        self.maze[3][18].set(up=False, down=False, left=True, right=True)
        self.maze[4][18].set(up=True, down=False, left=True, right=True)
        self.maze[5][18].set(up=False, down=False, left=True, right=True)
        self.maze[6][18].set(up=False, down=False, left=True, right=False)
        self.maze[7][18].set(up=False, down=False, left=False, right=False)
        self.maze[8][18].set(up=False, down=False, left=False, right=False)

        # bottom right
        self.maze[10][10].set(up=False, down=True, left=False, right=True)
        self.maze[11][10].set(up=False, down=False, left=True, right=True)
        self.maze[12][10].set(up=False, down=False, left=True, right=True)
        self.maze[13][10].set(up=False, down=True, left=True, right=True)
        self.maze[14][10].set(up=True, down=False, left=True, right=True)
        self.maze[15][10].set(up=False, down=True, left=True, right=True)
        self.maze[16][10].set(up=False, down=False, left=True, right=True)
        self.maze[17][10].set(up=False, down=False, left=True, right=False)
        self.maze[18][10].set(up=False, down=True, left=False, right=False)

        self.maze[10][11].set(up=True, down=True,left=False, right=False)
        self.maze[11][11].set(up=False, down=False, left=False, right=False)
        self.maze[12][11].set(up=False, down=False, left=False, right=False)
        self.maze[13][11].set(up=True, down=True, left=False, right=False)
        self.maze[14][11].set(up=False, down=False, left=False, right=False)
        self.maze[15][11].set(up=True, down=True, left=False, right=False)
        self.maze[16][11].set(up=False, down=False, left=False, right=False)
        self.maze[17][11].set(up=False, down=False, left=False, right=False)
        self.maze[18][11].set(up=True, down=True, left=False, right=False)

        self.maze[10][12].set(up=True, down=False, left=False, right=True)
        self.maze[11][12].set(up=False, down=False, left=True, right=True)
        self.maze[12][12].set(up=False, down=True, left=True, right=False)
        self.maze[13][12].set(up=True, down=False, left=False, right=False)
        self.maze[14][12].set(up=False, down=False, left=False, right=False)
        self.maze[15][12].set(up=True, down=True, left=False, right=False)
        self.maze[16][12].set(up=False, down=False, left=False, right=True)
        self.maze[17][12].set(up=False, down=False, left=True, right=True)
        self.maze[18][12].set(up=True, down=True, left=True, right=False)

        self.maze[10][13].set(up=False, down=False, left=False, right=False)
        self.maze[11][13].set(up=False, down=False, left=False, right=False)
        self.maze[12][13].set(up=True, down=True, left=False, right=False)
        self.maze[13][13].set(up=False, down=False, left=False, right=True)
        self.maze[13][13].goal = True
        self.maze[14][13].set(up=False, down=False, left=True, right=True)
        self.maze[15][13].set(up=True, down=True, left=True, right=True)
        self.maze[16][13].set(up=False, down=False, left=True, right=True)
        self.maze[17][13].set(up=False, down=False, left=True, right=False)
        self.maze[18][13].set(up=True, down=True, left=False, right=False)

        self.maze[10][14].set(up=False, down=True, left=False, right=True)
        self.maze[11][14].set(up=False, down=False, left=True, right=True)
        self.maze[12][14].set(up=True, down=False, left=True, right=True)
        self.maze[13][14].set(up=False, down=False, left=True, right=True)
        self.maze[14][14].set(up=False, down=False, left=True, right=False)
        self.maze[15][14].set(up=True, down=True, left=False, right=False)
        self.maze[16][14].set(up=False, down=False, left=False, right=True)
        self.maze[17][14].set(up=False, down=False, left=True, right=True)
        self.maze[18][14].set(up=True, down=True, left=True, right=False)

        self.maze[10][15].set(up=True, down=True, left=False, right=False)
        self.maze[11][15].set(up=False, down=False, left=False, right=True)
        self.maze[12][15].set(up=False, down=False, left=True, right=True)
        self.maze[13][15].set(up=False, down=True, left=True, right=True)
        self.maze[14][15].set(up=False, down=False, left=True, right=True)
        self.maze[15][15].set(up=True, down=True, left=True, right=True)
        self.maze[16][15].set(up=False, down=False, left=True, right=True)
        self.maze[17][15].set(up=False, down=False, left=True, right=False)
        self.maze[18][15].set(up=True, down=True, left=False, right=False)

        self.maze[10][16].set(up=True, down=True, left=False, right=True)
        self.maze[11][16].set(up=False, down=False, left=True, right=True)
        self.maze[12][16].set(up=False, down=True, left=True, right=False)
        self.maze[13][16].set(up=True, down=True, left=False, right=False)
        self.maze[14][16].set(up=False, down=True, left=False, right=False)
        self.maze[15][16].set(up=True, down=True, left=False, right=False)
        self.maze[16][16].set(up=False, down=False, left=False, right=False)
        self.maze[17][16].set(up=False, down=False, left=False, right=False)
        self.maze[18][16].set(up=True, down=True, left=False, right=False)

        self.maze[10][17].set(up=True, down=True, left=False, right=False)
        self.maze[11][17].set(up=False, down=False, left=False, right=False)
        self.maze[12][17].set(up=True, down=True, left=False, right=False)
        self.maze[13][17].set(up=True, down=False, left=False, right=False)
        self.maze[14][17].set(up=True, down=True, left=False, right=False)
        self.maze[15][17].set(up=True, down=False, left=False, right=True)
        self.maze[16][17].set(up=False, down=False, left=True, right=True)
        self.maze[17][17].set(up=False, down=False, left=True, right=True)
        self.maze[18][17].set(up=True, down=False, left=True, right=False)

        self.maze[10][18].set(up=True, down=False, left=False, right=False)
        self.maze[11][18].set(up=False, down=False, left=False, right=False)
        self.maze[12][18].set(up=True, down=False, left=False, right=True)
        self.maze[13][18].set(up=False, down=False, left=True, right=True)
        self.maze[14][18].set(up=True, down=False, left=True, right=True)
        self.maze[15][18].set(up=False, down=False, left=True, right=True)
        self.maze[16][18].set(up=False, down=False, left=True, right=False)
        self.maze[17][18].set(up=False, down=False, left=False, right=False)
        self.maze[18][18].set(up=False, down=False, left=False, right=False)

        # top right
        self.maze[10][0].set(up=False, down=True, left=False, right=True)
        self.maze[11][0].set(up=False, down=False, left=True, right=True)
        self.maze[12][0].set(up=False, down=False, left=True, right=True)
        self.maze[13][0].set(up=False, down=True, left=True, right=True)
        self.maze[14][0].set(up=False, down=False, left=True, right=True)
        self.maze[15][0].set(up=False, down=True, left=True, right=True)
        self.maze[16][0].set(up=False, down=False, left=True, right=True)
        self.maze[17][0].set(up=False, down=False, left=True, right=False)
        self.maze[18][0].set(up=False, down=True, left=False, right=False)

        self.maze[10][1].set(up=True, down=True,left=False, right=False)
        self.maze[11][1].set(up=False, down=False, left=False, right=False)
        self.maze[12][1].set(up=False, down=False, left=False, right=False)
        self.maze[13][1].set(up=True, down=True, left=False, right=False)
        self.maze[14][1].set(up=False, down=False, left=False, right=False)
        self.maze[15][1].set(up=True, down=True, left=False, right=False)
        self.maze[16][1].set(up=False, down=False, left=False, right=False)
        self.maze[17][1].set(up=False, down=False, left=False, right=False)
        self.maze[18][1].set(up=True, down=True, left=False, right=False)

        self.maze[10][2].set(up=True, down=False, left=False, right=True)
        self.maze[11][2].set(up=False, down=False, left=True, right=True)
        self.maze[12][2].set(up=False, down=True, left=True, right=False)
        self.maze[13][2].set(up=True, down=False, left=False, right=False)
        self.maze[14][2].set(up=False, down=False, left=False, right=False)
        self.maze[15][2].set(up=True, down=True, left=False, right=False)
        self.maze[16][2].set(up=False, down=False, left=False, right=True)
        self.maze[17][2].set(up=False, down=False, left=True, right=True)
        self.maze[18][2].set(up=True, down=True, left=True, right=False)

        self.maze[10][3].set(up=False, down=False, left=False, right=False)
        self.maze[11][3].set(up=False, down=False, left=False, right=False)
        self.maze[12][3].set(up=True, down=True, left=False, right=False)
        self.maze[13][3].set(up=False, down=False, left=False, right=True)
        self.maze[14][3].set(up=False, down=False, left=True, right=True)
        self.maze[15][3].set(up=True, down=True, left=True, right=True)
        self.maze[16][3].set(up=False, down=False, left=True, right=True)
        self.maze[17][3].set(up=False, down=False, left=True, right=False)
        self.maze[18][3].set(up=True, down=True, left=False, right=False)

        self.maze[10][4].set(up=False, down=True, left=True, right=True)
        self.maze[11][4].set(up=False, down=False, left=True, right=True)
        self.maze[12][4].set(up=True, down=False, left=True, right=True)
        self.maze[13][4].set(up=False, down=False, left=True, right=True)
        self.maze[14][4].set(up=False, down=False, left=True, right=False)
        self.maze[15][4].set(up=True, down=True, left=False, right=False)
        self.maze[16][4].set(up=False, down=False, left=False, right=True)
        self.maze[17][4].set(up=False, down=False, left=True, right=True)
        self.maze[18][4].set(up=True, down=True, left=True, right=False)

        self.maze[10][5].set(up=True, down=True, left=False, right=False)
        self.maze[11][5].set(up=False, down=False, left=False, right=True)
        self.maze[12][5].set(up=False, down=False, left=True, right=True)
        self.maze[13][5].set(up=False, down=True, left=True, right=True)
        self.maze[14][5].set(up=False, down=False, left=True, right=True)
        self.maze[15][5].set(up=True, down=True, left=True, right=True)
        self.maze[16][5].set(up=False, down=False, left=True, right=True)
        self.maze[17][5].set(up=False, down=False, left=True, right=False)
        self.maze[18][5].set(up=True, down=True, left=False, right=False)

        self.maze[10][6].set(up=True, down=True, left=False, right=True)
        self.maze[11][6].set(up=False, down=False, left=True, right=True)
        self.maze[12][6].set(up=False, down=True, left=True, right=False)
        self.maze[13][6].set(up=True, down=True, left=False, right=False)
        self.maze[14][6].set(up=False, down=True, left=False, right=False)
        self.maze[15][6].set(up=True, down=True, left=False, right=False)
        self.maze[16][6].set(up=False, down=False, left=False, right=False)
        self.maze[17][6].set(up=False, down=False, left=False, right=False)
        self.maze[18][6].set(up=True, down=True, left=False, right=False)

        self.maze[10][7].set(up=True, down=True, left=False, right=False)
        self.maze[11][7].set(up=False, down=False, left=False, right=False)
        self.maze[12][7].set(up=True, down=True, left=False, right=False)
        self.maze[13][7].set(up=True, down=False, left=False, right=False)
        self.maze[14][7].set(up=True, down=True, left=False, right=False)
        self.maze[15][7].set(up=True, down=False, left=False, right=True)
        self.maze[16][7].set(up=False, down=False, left=True, right=True)
        self.maze[17][7].set(up=False, down=False, left=True, right=True)
        self.maze[18][7].set(up=True, down=False, left=True, right=False)

        self.maze[10][8].set(up=True, down=False, left=False, right=False)
        self.maze[11][8].set(up=False, down=False, left=False, right=False)
        self.maze[12][8].set(up=True, down=False, left=False, right=True)
        self.maze[13][8].set(up=False, down=False, left=True, right=True)
        self.maze[14][8].set(up=True, down=True, left=True, right=True)
        self.maze[15][8].set(up=False, down=False, left=True, right=True)
        self.maze[16][8].set(up=False, down=False, left=True, right=False)
        self.maze[17][8].set(up=False, down=False, left=False, right=False)
        self.maze[18][8].set(up=False, down=False, left=False, right=False)