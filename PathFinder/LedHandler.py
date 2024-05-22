try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from FakeGpio import GPIO


# LED actuator
class LedActuator:
    # In Board mode, the LED pin numbers
    def __init__(self, led):
        GPIO.setup(led, GPIO.OUT, initial=GPIO.LOW)
        self.led = led

    def set(self, led):
        GPIO.output(self.led, GPIO.HIGH if led else GPIO.LOW)
