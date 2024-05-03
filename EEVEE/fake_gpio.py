import random

class FakePwm:
    def start(self, val: int):
        pass
    def ChangeDutyCycle(self, val: int):
        pass

class GPIO:
    BOARD = 0
    IN = 0
    OUT = 1
    LOW = 0
    HIGH = 1
    BOTH = 2
    FALLING = 0
    RISING = 1
    PUD_DOWN = 0

    pin_values = [0]*40

    @classmethod
    def setwarnings(cls, b: bool):
        pass

    @classmethod
    def setmode(cls, some_val: int):
        pass

    @classmethod
    def setup(cls, pin_number: int, mode: int, initial: int = LOW, pull_up_down: int = None):
        pass

    @classmethod
    def PWM(cls, something, frequency: int) -> FakePwm:
        return FakePwm()

    @classmethod
    def output(cls, something, state: int):
        pass

    @classmethod
    def input(cls, pin_number: int) -> bool:
        if random.random() > 0.99:
            GPIO.pin_values[pin_number] = 1 - GPIO.pin_values[pin_number]
        return GPIO.pin_values[pin_number]

    @classmethod
    def add_event_detect(cls, pin_number: int, state: int, func):
        pass

    @classmethod
    def add_event_detect(cls, pin_number: int, state: int, func):
        pass

    @classmethod
    def cleanup(cls):
        pass
