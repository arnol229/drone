#!/usr/bin/env python
try:
        import sys
        import spidev
        import time
        import os
        from RPi import GPIO
        import threading
        import smbus
        import socket
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
        Check if it is a pi and that i2c and SPI are Enabled
        if not, make file changes and reboot
        add a script on boot to reload this

        if module fails import, pip install it

        Check GPIO pins?

        Make Motor class and attach to the drone

        Everything else.

    """
    def __init__(self,version):
        ## Delay in checking input
        # new connection method:
        if version not in [1,2]:
            raise Exception("Version 1 or 2 available only")

        if version == 1:
            self.__delay = .02
            GPIO.setmode(GPIO.BCM)
            self.__calibration_retry = 0
            self.__calibration_tries = 0

            ########## SPI ##########
            print "Initializing SPI port"

            self.__spi = spidev.SpiDev()
            self.__spi.open(0,0)

            #ADC channel on the ADC chip#
            self.__joy_x_channel = 0
            self.__joy_y_channel = 1
            self.__joy_swt_channel = 2


            ########## LED ##########
            self.__led_r_pin = 24
            self.__led_b_pin = None
            self.__led_g_pin = None

            GPIO.setup(self.__led_r_pin, GPIO.OUT)
            GPIO.output(self.__led_r_pin, GPIO.HIGH)


            ########## JOYSTICK ##########
            print "Initializing joystick"

            self.joy_x_val = None
            self.joy_y_val = None
            self.joy_swt_val = None

            self.__joystick_thread = threading.Thread(target=self.joy_input,name='joy_thread')
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
            self.__starting_freq = 1000
            self.__starting_dc = 50

            GPIO.setup(self.__motor1_pin, GPIO.OUT)#left motor
            GPIO.setup(self.__motor2_pin, GPIO.OUT)#right motor
            self.motor_1_pwm = GPIO.PWM(self.__motor1_pin,self.__starting_freq)
            self.motor_2_pwm = GPIO.PWM(self.__motor2_pin,self.__starting_freq)

            raw_input("Press enter to enter motor calibration")
            while True:
                if not self.calibrate_motors():
                    self.motor_2_pwm.stop()
                    self.motor_1_pwm.stop()
                    print "Restarting Calibration..."
                    continue
                print "Calibration complete."
                break
            ############################

        elif version == 2:
            self.DroneIP="192.168.1.74"
            try:
                socket.socket().connect((self.DroneIP, 21))
                socket.socket().close()
            except:
                print "Drone is not online"
                sys.exit(9)

            #send the first four initial-commands to the drone
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Open network connection
            self.__sock.setblocking(0)                                      # Network should not block
            self.__sendrawmsg("\r")                                         # Wakes up command port
            time.sleep(0.01)
            self.__sendrawmsg("AT*PMODE=1,2\rAT*MISC=2,2,20,2000,3000\r")




        print "Ready"

    def __sendrawmsg(self, msg):
        try:        self.__keepalive.cancel()
        except:     pass
        if self.showCommands:
            if msg.count("COMWDG") < 1: print msg
        self.__sock.sendto(msg, (self.DroneIP, self.CmdPort))
        self.__keepalive = threading.Timer(0.1, self.__heartbeat)
        self.__keepalive.start()

    #should these 2 methods be inside calibrate_motors?
    def print_joy(self):
            while True:
                sys.stdout.write("\r" + str(self.__joy_x_channel))
                sys.stdout.write("\033[K")
                sys.stdout.flush()
    
    def retry_calibration(self):
        while True:
            if not self.joy_swt_val:
                self.__calibration_tries = 0
                print "\n\nRetry! The battery should be replugged back in!"


    def calibrate_motors(self):
        """
        Motor Calibration
        ----------
        (Should do something like)
        1) point joystick to the highest point in the throttle
        and press enter to sync with the ESC
        2) point joystick to the lowest point in the throttle
        and press enter to sync with the ESC
        3) Return to the center and press enter to start flying!
        ===
        IF you want to start over:
        unplug the battery click the joystick in
        """
        print self.calibrate_motors.__doc__
        tries = 0

        #def print_joy():
        #    while True:
        #        sys.stdout.write("\r" + str(self.__joy_x_channel))
        #        sys.stdout.write("\033[K")
        #        sys.stdout.flush()

        #def retry_calibration(self):
        #    while True:
        #        if not self.joy_swt_val:
        #        self.__calibration_tries = 0
        #        raw_input("Retry! Hit enter to verify that the battery has been replugged back in")

        ## start thread for printing out the joystick reading and listening for joystick switch to retry
        joy_display = threading.Thread(target=self.print_joy,name="print_joy")
        retry = threading.Thread(target=self.calibration_retry,name="calibration_retry")
        
        retry.start()
        joy_display.start()
        raw_input("\nSelect Lowest Throttle press enter to start PWM:")
        print "this is hardcoded to be 50 percent right now"
        self.motor_2_pwm.start(50)
        self.motor_2_pwm.start(50)
        while tries <= 3:
            tries += 1
            #retry.start()
            raw_input("\nPress enter to set PWM:")
            #retry.stop()
            print# "\n"
            # \/ shouldnt be needed if the value stays when joy_display is stoppped?
            # print str(self.joy_x_val)
            if not self.set_duty_cycle(pwm=self.joy_x_val):
                joy_display.stop()
                retry.stop()
                print "Replug in the battery and try again!"
                return False
        retry.stop()
        joy_display.stop()
        return True

    def set_duty_cycle(self,pwm=None):
        while True:
            if pwm:
                try:
                    pwm = abs(int(pwm))
                    if pwm > 0 and pwm < 100:
                        self.motor_2_pwm.ChangeDutyCycle(pwm)
                        self.motor_1_pwm.ChangeDutyCycle(pwm)
                        return True
                    else:
                        print "Duty Cycle must be between in range 0-100"
                        return False
                except ValueError:
                    print "Psssst. You are supposed to enter a number!"
                    return False

            else:
                print "You can type cancel to escape"
                pwm = raw_input("Set Duty Cycle to: ")
                if pwm == "cancel":
                    print "--Cancelled--"
                    return False

    def read_adc(self, channel):
        adc = self.__spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data

    def joy_input(self):
        """
        Continually reads Reads the joy stick values to the drone and delays
        Initiated as 'joy_thread' in drone start-up
        """
        while True:
            self.joy_x_val = ((self.read_adc(self.__joy_x_channel)/10)-100)*-1
            self.joy_y_val = ((self.read_adc(self.__joy_y_channel)/10)-100)*-1
            self.joy_swt_val = self.read_adc(self.__joy_swt_channel)
            time.sleep(self.__delay) if self.__delay else 0

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

                ## this math is all kinds of screwed up
                if self.joy_x_val > 55:
                    adj_val = abs(self.joy_x_val - speed_val)
                    pwm_val = speed_val + adj_val
                    if pwm_val > 90:
                        pwm_val = 90
                    elif pwm_val < 20:
                        pwm_val = 10
                    text = "\rleft: " + str(pwm_val) + " right: " + str(speed_val)
                    self.motor_1_pwm.ChangeDutyCycle(pwm_val)
                    self.motor_2_pwm.ChangeDutyCycle(speed_val)

                elif self.joy_x_val < 45:
                    adj_val = abs(self.joy_x_val - speed_val)
                    pwm_val = speed_val + adj_val
                    if pwm_val > 90:
                        pwm_val = 90
                    elif pwm_val < 20:
                        pwm_val = 10
                    text = "\rleft: " + str(speed_val) + " right: " + str(pwm_val)
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
