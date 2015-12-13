#!/usr/bin/env python
try:
        import spidev
        import time
        import os
        from RPi import GPIO
        import threading
except Exception as e:
    print("Error importing module: {0}".format(str(e)))
    exit()

class Drone:
    def __init__(self):
        ## SPI interface for reading the ADC
        print "Initializing SPI port"
        self.__spi=spidev.SpiDev()
        self.__spi.open(0,0)
        ## ADC channel on the ADC chip
        self.__joy_x_channel=0
        self.__joy_y_channel=1
        self.__joy_swt_channel=2

        # Joy stick values
        self.joy_x_val=None
        self.joy_y_val=None
        self.joy_swt_val=None

        ## Delay in checking input
        self.__delay=.005

        ## Motor Details
        self.__motor_pwm_pin=21
        self.__starting_freq=1000
        self.__starting_dc=50

        ## LED Details
        self.__led_r=None
        self.__led_b=None
        self.__led_g=None

        ## Initializing motor
        print "Initializing motor"
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__motor_pwm_pin, GPIO.OUT)
        self.pwm=GPIO.PWM(self.__motor_pwm_pin,self.__starting_freq)
        self.pwm.start(50)

    def read_adc(self, channel):
        adc = self.__spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data

    def joy_input(self):
        while True:
            self.joy_x_val = ((self.read_adc(self.__joy_x_channel)/10)-100)*-1
            self.joy_y_val = ((self.read_adc(self.__joy_y_channel)/10)-100)*-1
            self.joy_swt_val = self.read_adc(self.__joy_swt_channel)
            time.sleep(self.__delay)

    def fly(self):
        try:
            joystick_thread = threading.Thread(target=self.joy_input)
            joystick_thread.daemon = True
            joystick_thread.start()

            while True:
                print "x:{0} | y:{1} | click:{2}\r".format(
                        self.joy_x_val,
                        self.joy_y_val,
                        self.joy_swt_val),
        except KeyboardInterrupt:
            print
            print "Exiting flying mode"
        except Exception as e:
            print "error while flying: {0}".format(str(e))

if __name__ == "__main__":
    print "Drone initializing..."
    drone = Drone()
    time.sleep(2)
    print "starting to fly"
    drone.fly()
    print "cleaning up"
    GPIO.cleanup()
