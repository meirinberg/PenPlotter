import gc
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array

class Joystick: 
       '''
       Configures joystick to interact with Nucleo to get proper ADC
       and button press values. Joystick values range from 0-4096 with
       the undisturbed state reading 2048.
       '''
       def __init__(self):
          # Joystick Pin Connections
          self.XPIN = Pin(Pin.cpu.A5, mode=Pin.IN) # Conigures pin as input
          self.YPIN = Pin(Pin.cpu.A6, mode=Pin.IN) # Conigures pin as input
          self.BUTTONPIN = Pin(Pin.cpu.C5, mode=Pin.IN, pull=pyb.Pin.PULL_UP)
          self.x = pyb.ADC(self.XPIN)    # Creates an analog object from a pin
          self.y = pyb.ADC(self.YPIN)    # Creates an analog object from a pin
    
       def readX(self):
           # Returns x data values
          return self.x.read()
    
       def readY(self):
           #Returns y data values
          return self.y.read()
    
       def readSwitch(self):
          return not self.BUTTONPIN.value() # If this returns true, the switch is pressed down. Otherwise, the switch is not pressed.