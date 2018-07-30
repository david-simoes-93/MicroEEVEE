#!/usr/bin/python3

import RPi.GPIO as GPIO
import time, sys, os, threading


# https://github.com/dmeziere/rpi-hc-sr04
# https://github.com/rhiller/pi-distance

# UltraSound sensor
class USSensor:
    # In Board mode, the TRIG and ECHO pin numbers
    def __init__(self, trig, echo):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)

        self.limit = 0.5
        self.echo = echo
        self.trig = trig

    # Get a distance measure (m) or None. Takes up to 1s (but usually less than 0.5s)
    def get(self):
        # Send a trigger for 1ms
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.001)
        start = time.time()
        GPIO.output(self.trig, GPIO.LOW)

        ping, pong = start, start

        # Wait up to LIMIT for the ping to start
        while ping - start < self.limit:
            ping = time.time()
            if GPIO.input(self.echo):
                break
        else:
            return None

        # Wait up to LIMIT for the pong to return
        while pong - ping < self.limit:
            pong = time.time()
            if not GPIO.input(self.echo):
                break
        else:
            return None

        # speed of sound is 343.4m/s, 20ºC from WolframAlpha
        # since we have a 2-trip movement, we divide that by 2, giving us 171.7m/s
        return (pong - ping) * 171.7


# UltraSound sensor
class USSensorCallbacks:
    # In Board mode, the TRIG and ECHO pin numbers
    def __init__(self, trig, echo):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.add_event_detect(echo, GPIO.BOTH, self.read_ping)
        self.trig_event = threading.Event()

        self.limit = 0.5
        self.echo = echo
        self.trig = trig
        self.dist = None
        self.ping = None
        self.waiting_for_pong = False

    # Get a distance measure (m) or None. Takes up to 1s (but usually less than 0.5s)
    def get(self):
        #self.dist = None
        #self.waiting_for_pong = False

        # Send a trigger for 1ms
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.001)
        #start = time.time()
        GPIO.output(self.trig, GPIO.LOW)

        # Wait up to 10ms for the ping to start
        #self.ping = start
        #while ping - start < 0.010:
        #    ping = time.time()
        #    if GPIO.input(self.echo):
        #        self.trig_event.clear()
        #        break
        #else:
        #    return None

        # wait for echo
        #self.trig_event.wait(timeout=self.limit)
        #return self.dist

    def read_ping(self, _):
        if GPIO.input(self.echo):
            self.ping = time.time()
            #self.waiting_for_pong = True
        else:
            # speed of sound is 343.4m/s, 20ºC from WolframAlpha
            # since we have a 2-trip movement, we divide that by 2, giving us 171.7m/s
            self.dist = (time.time() - self.ping) * 171.7
            print(self.dist)
            self.get()
            # wake up trigger thread
            #self.waiting_for_pong = False
            #self.trig_event.set()


# UltraSound sensor
class USSensorInterrupts:
    # In Board mode, the TRIG and ECHO pin numbers
    def __init__(self, trig, echo):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)

        self.limit = 500
        self.echo = echo
        self.trig = trig

    # Get a distance measure (m) or None. Takes up to 1s (but usually less than 0.5s)
    def get(self):
        # Send a trigger for 1ms
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trig, GPIO.LOW)

        # Wait up to LIMIT for the ping to start
        # THIS IS A PROBLEM: thread may fall asleep after the rising edge
        ping = GPIO.wait_for_edge(self.echo, GPIO.RISING, timeout=self.limit)
        if ping is None:
            return 0
        ping = time.time()

        # Wait up to LIMIT for the pong to return
        # THIS IS A PROBLEM: thread may fall asleep after the falling edge
        pong = GPIO.wait_for_edge(self.echo, GPIO.FALLING, timeout=self.limit)
        if pong is None:
            return None

        # speed of sound is 343.4m/s, 20ºC from WolframAlpha
        # since we have a 2-trip movement, we divide that by 2, giving us 171.7m/s
        return (time.time() - ping) * 171.7


def main():
    # Asynchronous, no callbacks
    #us0 = USSensorInterrupts(12, 16)
    #while True:
    #    print(us0.get())

    # Asynchronous, using callbacks
    us0 = USSensorCallbacks(12, 16)
    us0.get()
    #while True:
    #    print(us0.get())

    # Synchronous, no interrupts
    us0 = USSensor(12, 16)
    while True:
        print(us0.get())


main()
