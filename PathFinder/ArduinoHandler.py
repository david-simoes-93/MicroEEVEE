import serial
import random

from Utils import MEDIAN_SIZE


class EmptyArduino(object):
    def __init__(self, simulator):
        self.ir0 = 0
        self.ir1 = 0
        self.button0 = False
        self.button1 = False
        self.ground0 = False
        self.ground1 = False
        self.ground2 = False
        self.ground3 = False
        self.ground4 = False
        self.m1_encoder = 0
        self.m2_encoder = 0

        self.simulator = simulator

    def get(self):
        self.button0 = random.random() > 0.5
        self.button1 = random.random() > 0.5

        self.simulator.update_loc()

        [self.ground0, self.ground1, self.ground2, self.ground3, self.ground4] = self.simulator.get_ground()

        [self.m1_encoder, self.m2_encoder] = self.simulator.get_encoder()

        return True

    def get_ground_average(self):
        ground_sensors = self.get_ground_sensors()
        return sum(ground_sensors)/len(ground_sensors)

    def get_ground_sensors(self):
        return [self.ground0, self.ground1, self.ground2, self.ground3, self.ground4]

# Arduino handler
class ArduinoHandler:
    def __init__(self):

        # Arduino connection
        self.arduino = serial.Serial("/dev/ttyUSB0", 9600)
        self.arduino.flushInput()

        self.ir0 = 0
        self.ir1 = 0
        self.button0 = False
        self.button1 = False
        self.ground0 = False
        self.ground1 = False
        self.ground2 = False
        self.ground3 = False
        self.ground4 = False
        self.m1_encoder = 0
        self.m2_encoder = 0

        # Min message = 5.00;5.00;1;1;0;0;0;0;0;0;0;\r\n
        self.min_mess_size = 30

        self.arduino.readline()

        self.buf = [[0]*MEDIAN_SIZE, [0]*MEDIAN_SIZE]
        self.i = [0, 0]

    def get(self):
        try:
            line = self.arduino.readline()
        except:
            return False
        # just debugging line, eventually remove after confirming
        if len(line) < self.min_mess_size:
            print(f"min length {len(line)} is bugged for line {line}")
        sensors = line.decode().split(";")
        self.m1_encoder = 0
        self.m2_encoder = 0

        # read extra lines if they exist
        while self.arduino.in_waiting >= self.min_mess_size:
            print("Losing cycles... ", self.arduino.in_waiting)
            self.m1_encoder += float(sensors[9])
            self.m2_encoder += float(sensors[10])
            try:
                sensors = self.arduino.readline().decode().split(";")
            except:
                return False

        self.button0 = sensors[2] == '1'
        self.button1 = sensors[3] == '1'

        # true if line detected
        self.ground0 = sensors[4] == '1'
        self.ground1 = sensors[5] == '1'
        self.ground2 = sensors[6] == '1'
        self.ground3 = sensors[7] == '1'
        self.ground4 = sensors[8] == '1'

        self.m1_encoder += float(sensors[9])
        self.m2_encoder += float(sensors[10])

        # negate m1 encoder
        self.m1_encoder = -self.m1_encoder

        return True

    def clear(self):
        self.arduino.flushInput()
        self.arduino.readline()

    def median(self, newValue, index):
        aux = [None] * MEDIAN_SIZE

        k = self.i[index]
        self.buf[index][k] = newValue
        self.i[index] = (k + 1) % MEDIAN_SIZE

        for j in range(0, MEDIAN_SIZE):
            aux[j] = self.buf[index][j]

        aux.sort()  # sort(aux, MEDIAN_SIZE);
        return aux[int(MEDIAN_SIZE / 2)]

    def get_ground_average(self):
        ground_sensors = self.get_ground_sensors()
        return sum(ground_sensors)/len(ground_sensors)

    def get_ground_sensors(self):
        # ground1 is broken, always true
        return [self.ground0, self.ground1, self.ground2, self.ground3, self.ground4]
