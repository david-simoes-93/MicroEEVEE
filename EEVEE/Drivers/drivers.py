#python3

import spidev
import time

class AnalogSensors(object):
    #ADC needed

    def __init__(self):
        # distance sensors
        self.obstSensRight = 0
        self.obstSensFront = 0
        self.obstSensLeft = 0

        self.an6 = 0
        self.an7 = 0
        self.batteryVoltage = 0
        self.groundSensor = [1] # TODO
        # Position variables
        self.counterLeft = 0
        self.counterRight = 0
        #adc interface MCP3008
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)

    #TODO
#     def int median(int sensor, int newvalue);
# int average(int sensor, int newValue);
# int calcDistance(int adcValue);

    def readChannel(self, channelNr): #channel Nr como um define
        if channelNr > 7 or channelNr < 0:
            return -1
        r = self.spi.xfer2([1, 8 + channelNr << 4, 0])
        data = ((r[1] & 3) << 8) + r[2]
        return data


    # *****************************************
    # **************** Interrupts *************
    # *****************************************

    # TODO for counters




#**********************************************
M1_IN1 = LATBbits.LATB5
M1_IN2 = LATCbits.LATC13
M2_IN1 = LATBbits.LATB13
M2_IN2 = LATFbits.LATF3
STDBY = LATCbits.LATC14

LED1 = LATEbits.LATE0
LED2 = LATEbits.LATE1
LED3 = LATEbits.LATE2
LED4 = LATEbits.LATE3

PI = 3.141592654

# TODO find a way to do this as global variables
#define M1_FORWARD M1_IN1=1; M1_IN2=0
#define M1_REVERSE M1_IN1=0; M1_IN2=1

#define M2_FORWARD M2_IN1=0; M2_IN2=1
#define M2_REVERSE M2_IN1=1; M2_IN2=0

#define startButton() (!PORTBbits.RB3)
#define stopButton() (!PORTBbits.RB4) //*! get stop button status

#*****************************************************
# Robot dimensions
WHEEL2WHEEL_DIST = 165	# mm
WHEEL_DIAM	= 80.6	# mm
WHEEL_PER	= PI * WHEEL_DIAM

#*****************************************************
# Motor parameters
RPM_OUT	= 160		# RPM in the output shaft @ 9.6V
GEAR_RATIO = 10		# Gear box ratio
ENCODER_PULSES = 100	# Encoder, pulses per revolution

#*****************************************************
# Controller parameters
SAMPLING_T = 20		# [ms]
SP_MAX	= (RPM_OUT * GEAR_RATIO * ENCODER_PULSES * SAMPLING_T) / 60000
INTEGRAL_CLIP = 70				# Cliping value of the Integral component
KP_num = 7
KP_den=2	# Propotional constant
KI_num=1
KI_den=1	# Integral constant

    # define DIM_BAT_ARRAY	   128          // Dimension of the battery voltage array
    # define MEDIAN_SIZE	   7            // Median buffer size for obstacle sensor filtering
    # define AVERAGE_SIZE	   1            // Average buffer size for obstacle sensor filtering

class ControlEEVEE (object):
    def __init__(self):
        # TODO config everything : LEDS, ADC, BATTERY, BUTTONS, SENSORS, ENCODERS

        self.analogSensors = AnalogSensors()

        self.spLeft = 0
        self.spRight = 0
        self.xpos = 0.0
        self.ypos = 0.0
        self.theta = 0.0
        self.pwmRight = 0
        self.pwmLeft = 0

        pass

    def readAnalogSensors(self):
        # TODO
        # TODO  : Channels description
        # TODO : read
        self.analogSensors.readChannel()
        pass

    def readLineSensors(self, gain):
        # Input: gain[1...100] (100 -> maximum gain)
        # Default value is 0(gain=50)

        # TODO: no sensors for this yet
        pass

    def getRobotPos(self):
        # return (x, y, t)
        pass

    def setRobotPos(self, x, y, t):
        pass

    def updateBatteryVoltage(self):
        # - update battery voltage (average of the last DIM_BAT_ARRAY readings)
        # - returned value is multiplied by 10 (max. value is 101, i.e. 10,1 V)
        pass

    # TODO SERVO

    def led(self, ledNr, value):
        pass

    def leds(self, value):
        pass

    def setSP2(self, spL, spR):
        pass

    def setPWM2(self, pwmL, pwmR):
        pass

    def setVel2(self, velL, velR):
        pass

    def _readEncoders(self, encLeft, encRight):
        #DisableInterrupts();
        self.encLeft = -self.analogSensors.counterLeft
        self.encRight = self.analogSensors.counterRight
        self.analogSensors.counterLeft = 0
        self.analogSensors.counterRight = 0
        #EnableInterrupts();

    # PID controller.
    # THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE ROUTINE(T >= 10 ms)
    # DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE(MAIN).DOING THAT MAY
    # PERMANENTLY DAMAGE THE MOTOR DRIVE.
    def _pid(self, spMLeft, encMLeft, spMRight, encMRight):
        pass

    # CAUTION:
    # THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE ROUTINE(T >= 10 ms)
    # DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE(MAIN).DOING THAT MAY
    # PERMANENTLY DAMAGE THE MOTOR DRIVE.
    def _actuateMotors(self, pwmL, pwmR):
        pass

    def updateLocalization(self, encLeft, encRight):
        pass

    #*****************************************
    #**************** Interrupts *************
    #*****************************************

    #TODO