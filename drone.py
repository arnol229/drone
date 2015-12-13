#!/usr/bin/env python
from __future__ import print_function
try:
        import spidev
        import time
        import os
        from RPi import GPIO
except Exception as e:
    print("messed up: {0}".format(str(e)))
    exit()

class Drone:
    def __init__(self):
        ## SPI interface for reading the ADC
        print("Initializing SPI port")
        self.__spi=spidev.SpiDev()
        self.__spi.open(0,0)
        ## ADC channel on the ADC chip
        self.__joy_x_channel=0

        ## Delay in checking input
        self.delay=.5

        ## Motor Details
        self.__motor_pwm_pin=21
        self.__starting_freq=1000
        self.__starting_dc=50

        ## LED Details
        self.__led_r=None
        self.__led_b=None
        self.__led_g=None

        ## Initializing motor
        print("Initializing motor")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__motor_pwm_pin, GPIO.OUT)
        self.pwm=GPIO.PWM(self.__motor_pwm_pin,self.__starting_freq)
        self.pwm.start(50)

    def read_adc(self, channel):
        adc = self.__spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data

    def fly(self):
        try:
            while True:
                val = ((self.read_adc(self.__joy_x_channel)/10)-100)*-1
                print("--------------------------------------------\n"+"Value: {0}".format(val), end='\r')
        except KeyboardInterrupt:
            print("Exiting flying mode")
        except Exception as e:
            print("error while flying: {0}".format(str(e)))

    # def read_adc(adcnum):
    #     if((adcnum > 7) or (adcnum < 0)):
    #         return -1
    #     r = self.__spi.xfer2([1,(8+adcnum)<<4,0])
    #     adcout = ((r[1]&3 << 8) + r[2])
    #     return adcout


print("well its something")
if __name__ == "__main__":
    print ("Drone initializing...")
    drone = Drone()
    time.sleep(2)
    print ("starting to fly")
    drone.fly()
