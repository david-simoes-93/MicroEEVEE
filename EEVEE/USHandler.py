import RPi.GPIO as GPIO
import time
from Utils import MEDIAN_SIZE


# UltraSound sensor
class USSensor:
    # In Board mode, the TRIG and ECHO pin numbers
    def __init__(self, trig, echo):
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

        self.limit = 0.5
        self.echo = echo
        self.trig = trig

        self.time_since = 0
        self.state = 0
        self.buf = [None]*MEDIAN_SIZE
        self.i = 0

    def do_phase(self, dist):
        curr_time = time.time()
        time_elapsed = curr_time - self.time_since

        # if triggering, check time and wait for ping
        if self.state == 0:
            if time_elapsed > 0.001:
                self.ping()

        # if waiting for ping, check pin/time and wait for pong
        elif self.state == 1:
            if GPIO.input(self.echo):
                #if random.uniform(0,1)>0.99:
                self.pong()
            elif time_elapsed > self.limit:
                self.trigger()

        # if waiting for pong, check pin/time and trigger again
        elif self.state == 2:
            if not GPIO.input(self.echo):
                #if time_elapsed >= 0.2:
                self.trigger()
                #dist.value = min(time_elapsed * 171.7, 2)
                dist.value = self.median(min(time_elapsed * 171.7, 2))
            elif time_elapsed > self.limit:
                self.trigger()

    def trigger(self):
        GPIO.output(self.trig, GPIO.HIGH)
        self.time_since = time.time()
        self.state = 0

    def ping(self):
        GPIO.output(self.trig, GPIO.LOW)
        self.time_since = time.time()
        self.state = 1

    def pong(self):
        self.time_since = time.time()
        self.state = 2

    def median(self, newValue):

        aux= [None] *MEDIAN_SIZE

        k = self.i
        self.buf[k] = newValue
        self.i = (k + 1) % MEDIAN_SIZE

        for j in range(0,MEDIAN_SIZE):
            aux[j] = self.buf[j]

        aux.sort() #sort(aux, MEDIAN_SIZE);
        return aux[int(MEDIAN_SIZE / 2)]


def us_async(keep_running, echo0, trig0, dist0, echo1, trig1, dist1, echo2, trig2, dist2, echo3, trig3, dist3):
    us0 = USSensor(echo0, trig0)
    us1 = USSensor(echo1, trig1)
    us2 = USSensor(echo2, trig2)
    us3 = USSensor(echo3, trig3)

    us0.trigger()
    us1.trigger()
    us2.trigger()
    us3.trigger()

    while keep_running.value:
        us0.do_phase(dist0)
        us1.do_phase(dist1)
        us2.do_phase(dist2)
        us3.do_phase(dist3)
