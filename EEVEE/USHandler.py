import time
from multiprocessing import Process, Value
from enum import Enum

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from fake_gpio import GPIO

from Utils import MEDIAN_SIZE

PING_TIME_LIMIT = 0.5

class SensorState(Enum):
    IDLE = 0
    PINGING = 1
    PINGED = 2


# UltraSound sensor
class USSensor:
    # In Board mode, the TRIG and ECHO pin numbers
    def __init__(self, trig: int, echo: int):
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.echo = echo
        self.trig = trig

        self.time_since = 0
        self.state = SensorState.IDLE
        self.buf = [0] * MEDIAN_SIZE
        self.current_index = 0

    def do_phase(self, dist):
        curr_time = time.time()
        time_elapsed = curr_time - self.time_since

        # if triggering, check time and wait for ping
        if self.state == SensorState.IDLE:
            if time_elapsed > 0.001:
                self.ping()

        # if waiting for ping, check pin/time and wait for pong
        elif self.state == SensorState.PINGING:
            if GPIO.input(self.echo):
                # if random.uniform(0,1)>0.99:
                self.pong()
            elif time_elapsed > PING_TIME_LIMIT:
                self.trigger()

        # if waiting for pong, check pin/time and trigger again
        elif self.state == SensorState.PINGED:
            if not GPIO.input(self.echo):
                # if time_elapsed >= 0.2:
                self.trigger()
                # dist.value = min(time_elapsed * 171.7, 2)
                dist.value = self.median(min(time_elapsed * 171.7, 2))
            elif time_elapsed > PING_TIME_LIMIT:
                self.trigger()

    def trigger(self):
        GPIO.output(self.trig, GPIO.HIGH)
        self.time_since = time.time()
        self.state = SensorState.IDLE

    def ping(self):
        GPIO.output(self.trig, GPIO.LOW)
        self.time_since = time.time()
        self.state = SensorState.PINGING

    def pong(self):
        self.time_since = time.time()
        self.state = SensorState.PINGED

    def median(self, newValue):
        self.buf[self.current_index] = newValue
        self.current_index = (self.current_index + 1) % MEDIAN_SIZE

        aux = list(self.buf)
        aux.sort()
        return aux[int(MEDIAN_SIZE / 2)]


class UltrasoundHandler:
    def __init__(self, echo0, trig0, echo1, trig1, echo2, trig2, echo3, trig3):
        self.us_left_val = Value('f', 0)
        self.us_front_val = Value('f', 0)
        self.us_right_val = Value('f', 0)
        self.us_back_val = Value('f', 0)

        self.us_back = USSensor(echo0, trig0)
        self.us_right = USSensor(echo1, trig1)
        self.us_front = USSensor(echo2, trig2)
        self.us_left = USSensor(echo3, trig3)

        self.keep_running = Value('b', True)

    def start_async(self):
        # asynchronously update US sensors
        Process(target=self.us_async, args=(self.keep_running, self.us_back, self.us_back_val, self.us_right,
                self.us_right_val, self.us_front, self.us_front_val, self.us_left, self.us_left_val)).start()

    # each sensor's distance (in CM)
    def left(self):
        return self.us_left_val.value * 100

    def right(self):
        return self.us_right_val.value * 100

    def front(self):
        return self.us_front_val.value * 100

    def back(self):
        return self.us_back_val.value * 100

    def us_async(self, keep_running, us_back, dist_back, us_right, dist_right, us_front, dist_front, us_left, dist_left):
        us_back.trigger()
        us_right.trigger()
        us_front.trigger()
        us_left.trigger()

        while keep_running.value:
            us_back.do_phase(dist_back)
            us_right.do_phase(dist_right)
            us_front.do_phase(dist_front)
            us_left.do_phase(dist_left)
