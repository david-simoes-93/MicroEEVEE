#!/usr/bin/python3

import RPi.GPIO as GPIO
import time, sys, os

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
		while ping-start < self.limit:
			ping = time.time()
			if GPIO.input(self.echo):
				break
		else:
			return None

		# Wait up to LIMIT for the pong to return
		while pong-ping < self.limit:
			pong = time.time()
			if not GPIO.input(self.echo):
				break
		else:
			return None

		# speed of sound is 343.4m/s, 20ºC from WolframAlpha
		return (pong-ping)*343.4


def main():
	us0 = USSensor(12, 16)

	while True:
		print(us0.get())
		

main()