import serial


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

    def get(self):
        sensors = self.arduino.readline().decode().split(";")
        self.m1_encoder=0
        self.m2_encoder=0
        # read extra lines if they exist
        while self.arduino.in_waiting>0:
            print("Losing cycles...")
            self.m1_encoder += float(sensors[9])
            self.m2_encoder += float(sensors[10])
            sensors = self.arduino.readline().decode().split(";")

        self.ir0 = float(sensors[0])
        self.ir1 = float(sensors[1])

        self.button0 = sensors[2] is '1'
        self.button1 = sensors[3] is '1'

        self.ground0 = sensors[4] is '1'
        self.ground1 = sensors[5] is '1'
        self.ground2 = sensors[6] is '1'
        self.ground3 = sensors[7] is '1'
        self.ground4 = sensors[8] is '1'

        self.m1_encoder += float(sensors[9])
        self.m2_encoder += float(sensors[10])

