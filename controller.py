




class Controller:
    """
    Controller that reads Joystick inputs through the ADC (Analog to Digital Converter) Chip
    TODO:
    Gyro?

    """
    def __init__(self, parrot=False):

        #Joystick values
        self.__spi = spidev.SpiDev()
        self.__spi.open(0,0)

        self.left_joy_x_val = None
        self.left_joy_y_val = None
        self.left_joy_swt_val = None
        self.left_joy_x_channel = None
        self.left_joy_y_channel = None
        self.left_joy_swt_channel = None

        self.right_joy_x_val = None
        self.right_joy_y_val = None
        self.right_joy_swt_val = None
        self.right_joy_x_channel = None
        self.right_joy_y_channel = None
        self.right_joy_swt_channel = None

        self.__joystick_thread = threading.Thread(target=self.joy_input,name='joy_thread')
        self.__joystick_thread.daemon = True
        self.__joystick_thread.start()

        # Connection
        self.__lock = threading.Lock()
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Open network connection
        self.__sock.setblocking(0)                                      # Network should not block

        self.drone_ip = None
        self.cmd_port = None
        if parrot:
            self.showCommands = True
            self.__sendrawmsg("\r")                                         # Wakes up command port
            time.sleep(0.01)
            self.__sendrawmsg("AT*PMODE=1,2\rAT*MISC=2,2,20,2000,3000\r")
            self.__CmdCounter = 0


    def joy_input(self):
        """
        Continually reads Reads the joy stick values to the drone and delays
        Initiated as 'joy_thread' in drone start-up
        """
        while True:
            self.left_joy_x_val = ((self.read_adc(self.left_joy_x_channel)/10)-100)*-1
            self.left_joy_y_val = ((self.read_adc(self.left_joy_y_channel)/10)-100)*-1
            self.left_joy_swt_val = self.read_adc(self.left_joy_swt_channel)
            self.right_joy_x_val = ((self.read_adc(self.right_joy_x_channel)/10)-100)*-1
            self.right_joy_y_val = ((self.read_adc(self.right_joy_y_channel)/10)-100)*-1
            self.right_joy_swt_val = self.read_adc(self.right_joy_swt_channel)
            time.sleep(self.__delay) if self.__delay else 0
            
    def read_adc(self, channel):
        adc = self.__spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data

    def __sendrawmsg(self, msg):
        try:        self.__keepalive.cancel()
        except:     pass
        if self.showCommands:
            if msg.count("COMWDG") < 1: print msg
        self.__sock.sendto(msg, (self.drone_ip, self.cmd_port))
        self.__keepalive = threading.Timer(0.1, self.__heartbeat)
        self.__keepalive.start()

    def __heartbeat(self):
            # If the drone does not get a command, it will mutter after 50ms (CTRL watchdog / state[28] will set to 1)
            # and panic after 2 seconds and abort data-communication on port 5554 (then you have to initialize the network again).
            # Heartbeat will reset the watchdog and, by the way, the ACK_BIT (state[6], to accept any other AT*CONFIG command)
            # If mainthread isn't alive anymore (because program crashed or whatever), heartbeat will initiate the shutdown.
            # if str(threading.enumerate()).count("MainThread, stopped") or str(threading.enumerate()).count("MainThread")==0:
            #     print "oh no"
            # else:   self.at("COMWDG",[])
            # quick fix to just see what happens
            self.at("COMWDG",[])

    def at(self, command, params):
        self.__lock.acquire()
        paramLn = ""
        if params:  
            for p in params:
                if type(p)   == int:    paramLn += ","+str(p)
                elif type(p) == float:  paramLn += ","+str(struct.unpack("i", struct.pack("f", p))[0])
                elif type(p) == str:    paramLn += ",\""+p+"\""
        msg = "AT*"+command+"="+str(self.__CmdCounter)+paramLn+"\r"
        self.__CmdCounter += 1
        self.__sendrawmsg(msg)
        self.__lock.release()