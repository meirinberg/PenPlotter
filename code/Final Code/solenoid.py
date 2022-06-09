import gc
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array

class Solenoid: 
   '''
   Controls solenoid to turn on (pen down) or off (pen up)
   '''
   def __init__(self):
      # Pin to activate solid state relay when trigger by Nucleo
      self.PENPIN = Pin(Pin.cpu.A1, mode=Pin.OUT)

   def down(self):
       # Activates solenoid, pushing pen down
      self.PENPIN.high()

   def up(self):
       # Deactivates solenoid, pushing pen up
      self.PENPIN.low()

   def getValue(self):
      return not self.PENPIN.value()  # If this returns true, the pen is down. Otherwise, the pen is up.
