import gc
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array

class LED: 
       '''
       Controls LED light to turn on or off
       '''
       def __init__(self):
          # Joystick Pin Connections
          self.LEDPIN = Pin(Pin.cpu.C4, mode=Pin.OUT)
    
       def on(self):
           # Turns LED on
          self.LEDPIN.high()
    
       def off(self):
           # Turns LED off
          self.LEDPIN.low()
    
       def getValue(self):
          return not self.LEDPIN.value()  # If this returns true, the LED is ON. Otherwise, the LED is OFF.