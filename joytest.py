import spidev
import time
import os
import pigpio 
from RPi import GPIO
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)
 
# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data
 
# Define sensor channels
# (channels 3 to 7 unused)
swt_channel = 0
#vrx_channel = 1
#vry_channel = 2
 
# Define delay between readings (s)
delay = 0.5
#pi = pigpio.pi()
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
pwm=GPIO.PWM(21,1000)
pwm.start(50)
while True:
 
  # Read the joystick position data
 # vrx_pos = ReadChannel(vrx_channel)
  #vry_pos = ReadChannel(vry_channel)
 
  # Read switch state
  swt_val = (((ReadChannel(swt_channel)/10)-100)*-1)

  if swt_val > 90:
    swt_val = 90
#  pwm_val = (10*swt_val) + 1000
  pwm_val = swt_val
  if swt_val < 10:
    pwm_val = 1
#100-74-8>  
#0(off)-1000-x-2000
  
  # Print out results
  print "--------------------------------------------"
  print("Value: {0}".format(pwm_val))
  pwm.ChangeDutyCycle(pwm_val)  
  # Wait before repeating loop
  time.sleep(delay)
pwm.stop
GPIO.cleanup()
