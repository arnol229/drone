#!/usr/bin/env python
try:
        import sys
        import spidev
        import time
        import os
        from RPi import GPIO
        import threading
        import smbus
except Exception as e:
    print("Error importing module: {0}".format(str(e)))
    exit()

class Drone:
    """
    Drone Prototype
    ---
    Enabled:
    Gyro/Accel
    Joystick
    motor
    ======
    To-Do
    ======
        Enable motor
        Encorporate 2 motors
        Everything else.
    """
    def __init__(self):
        ## Delay in checking input
        self.__delay = .02

        ########## SPI ##########
        print "Initializing SPI port"

        self.__spi = spidev.SpiDev()
        self.__spi.open(0,0)

        #ADC channel on the ADC chip#
        self.__joy_x_channel = 0
        self.__joy_y_channel = 1
        self.__joy_swt_channel = 2


        ########## LED ##########
        self.__led_r = None
        self.__led_b = None
        self.__led_g = None


        ########## JOYSTICK ##########
        print "Initializing joystick"

        self.joy_x_val = None
        self.joy_y_val = None
        self.joy_swt_val = None

        self.__joystick_thread = threading.Thread(target=self.joy_input)
        self.__joystick_thread.daemon = True
        self.__joystick_thread.start()


        ########## GYRO ##########
        self.__bus = smbus.SMBus(1)
        self.__bus.write_byte_data(0x68, 0x6b, 0)

        self.gyro_x_val = None
        self.gyro_y_val = None
        self.gyro_z_val = None
        self.accel_x_val = None
        self.accel_y_val = None
        self.accel_z_val = None

        self.__gyro_thread = threading.Thread(target=self.gyro_input)
        self.__gyro_thread.daemon = True
        self.__gyro_thread.start()


        ########## MOTOR ##########
        print "Initializing motor"

        self.__motor1_pin = 21
        self.__motor2_pin = 20
        self.__starting_freq = 4000
        self.__starting_dc = 50

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__motor1_pin, GPIO.OUT)#left motor
        GPIO.setup(self.__motor2_pin, GPIO.OUT)#right motor
        self.motor_1_pwm = GPIO.PWM(self.__motor1_pin,self.__starting_freq)
        self.motor_2_pwm = GPIO.PWM(self.__motor2_pin,self.__starting_freq)
        #calibration should move to its own function
        self.motor_2_pwm.start(50)
        self.motor_1_pwm.start(50)
        ############################
        print "Ready"

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

    def gyro_input(self):
        def read_word(adr):
            high = self.__bus.read_byte_data(0x68, adr)
            low = self.__bus.read_byte_data(0x68, adr+1)
            val = (high << 8) + low
            return val

        def read_word_2c(adr):
            val = read_word(adr)
            if (val >= 0x8000):
                return -((65535 - val) + 1)
            else:
                return val
        while True:
            self.gyro_x_val = read_word_2c(0x43)
            self.gyro_y_val = read_word_2c(0x45)
            self.gyro_z_val = read_word_2c(0x47)

            self.accel_x_val = read_word_2c(0x3b)# / 16384.0
            self.accel_y_val = read_word_2c(0x3d)# / 16384.0
            self.accel_z_val = read_word_2c(0x3f)# / 16384.0
            time.sleep(self.__delay)

    def fly(self):
        try:
            while True:
                # joy_text = "\rJoystick --- (( x:{0} | y:{1} | click:{2} )) ".format(
                #         self.joy_x_val,
                #         self.joy_y_val,
                #         self.joy_swt_val)
                # josytick moves to the right, x input is lower
                # joystick moves to the left, x input is bigger
                speed_val = self.joy_y_val
                if speed_val >= 70:
                    speed_val = 70
                elif speed_val <=30:
                    speed_val = 30

                text = "\rstarting"

                if self.joy_x_val < 55:
                    adj_val = abs(self.joy_x_val - speed_val)
                    pwm_val = speed_val + adj_val
                    text = "\rleft: " + str(pwm_val) + " right: " + str(speed_val)
                    if pwm_val > 90:
                        pwm_val = 90
                    elif pwm_val < 20:
                        pwm_val = 10
                    self.motor_1_pwm.ChangeDutyCycle(pwm_val)
                    self.motor_2_pwm.ChangeDutyCycle(speed_val)

                elif self.joy_x_val > 45:
                    adj_val = abs(self.joy_x_val - speed_val)
                    pwm_val = speed_val + adj_val
                    text = "\rleft: " + str(speed_val) + " right: " + str(pwm_val)
                    if pwm_val > 90:
                        pwm_val = 90
                    elif pwm_val < 20:
                        pwm_val = 10
                    self.motor_2_pwm.ChangeDutyCycle(pwm_val)
                    self.motor_1_pwm.ChangeDutyCycle(speed_val)

                else:
                    self.motor_2_pwm.ChangeDutyCycle(speed_val)
                    self.motor_1_pwm.ChangeDutyCycle(speed_val)
                    text = "\rleft: " + str(speed_val) + " right: " + str(speed_val)
                # gyro_text = "Gyro --- (( x:{0} | y:{1} | z:{2} )) ".format(
                #         self.gyro_x_val,
                #         self.gyro_y_val,
                #         self.gyro_z_val)
                # accel_text = "Accel --- (( x:{0} | y:{1} | z:{2} ))".format(
                #         self.accel_x_val,
                #         self.accel_y_val,
                #         self.accel_z_val)
                sys.stdout.write(text)# + gyro_text + accel_text)\
                sys.stdout.write("\033[K")
                sys.stdout.flush()

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
