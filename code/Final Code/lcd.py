import gc
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array

class LCD:
   '''
   Configures LCD to interpret commands/sent lettering.
   '''
   # Define some device constants
   LCD_WIDTH = 16    # Maximum characters per line
   LCD_CHR = True
   LCD_CMD = False

   LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
   LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

   # Timing Constants
   E_PULSE = 0.0005
   E_DELAY = 0.0005

   def __init__(self):
      # LCD Pin Connections
      self.LCD_E = Pin(Pin.cpu.B9, mode=Pin.OUT)
      self.LCD_RS = Pin(Pin.cpu.C9, mode=Pin.OUT)
      self.RWPIN = Pin(Pin.cpu.B8, mode=Pin.OUT)
      self.LCD_D4 = Pin(Pin.cpu.C8, mode=Pin.OUT)
      self.LCD_D5 = Pin(Pin.cpu.A10, mode=Pin.OUT)
      self.LCD_D6 = Pin(Pin.cpu.B3, mode=Pin.OUT)
      self.LCD_D7 = Pin(Pin.cpu.B5, mode=Pin.OUT)
       
      self.RWPIN.low()
      self.lcd_byte(0x01,self.LCD_CMD) # 000001 Clear display -- This line may be unnessesary
      self.lcd_byte(0x33,self.LCD_CMD) # 110011 Initialise
      self.lcd_byte(0x32,self.LCD_CMD) # 110010 Initialise
      self.lcd_byte(0x06,self.LCD_CMD) # 000110 Cursor move direction
      self.lcd_byte(0x0C,self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
      self.lcd_byte(0x28,self.LCD_CMD) # 101000 Data length, number of lines, font size
      self.lcd_byte(0x01,self.LCD_CMD) # 000001 Clear display
      time.sleep(self.E_DELAY)

   def lcd_byte(self, bits, mode):
      # Send byte to data pins
      # bits = data
      # mode = True  for character
      #        False for command
      if mode:
         self.LCD_RS.high() # RS
      else:
         self.LCD_RS.low() # RS
      upper = ((bits>>4)&0x0f);

      # High bits
      self.LCD_D4.low()
      self.LCD_D5.low()
      self.LCD_D6.low()
      self.LCD_D7.low()
      if upper&0b00000001==0b00000001:
         self.LCD_D4.high()
      if upper&0b00000010==0b00000010:
         self.LCD_D5.high()
      if upper&0b00000100==0b00000100:
         self.LCD_D6.high()
      if upper&0b00001000==0b00001000:
         self.LCD_D7.high()
        
      # Toggle 'Enable' pin
      self.lcd_toggle_enable()
      lower = ((bits)&0x0f);

      # Low bits
      self.LCD_D4.low()
      self.LCD_D5.low()
      self.LCD_D6.low()
      self.LCD_D7.low()
      if lower&0b00000001==0b00000001:
         self.LCD_D4.high()
      if lower&0b00000010==0b00000010:
         self.LCD_D5.high()
      if lower&0b00000100==0b00000100:
         self.LCD_D6.high()
      if lower&0b00001000==0b00001000:
         self.LCD_D7.high()
        
      # Toggle 'Enable' pin
      self.lcd_toggle_enable()

   def lcd_toggle_enable(self):
      # Toggle enable
      time.sleep(self.E_DELAY)
      self.LCD_E.high()
      time.sleep(self.E_PULSE)
      self.LCD_E.low()
      time.sleep(self.E_DELAY)

   def lcd_string(self, message,line):
      # Send string to display
      MESSAGE_SPACE = self.LCD_WIDTH-len(message)
      i=0
      while i<MESSAGE_SPACE:
         message = message + " "
         i+=1 
      self.lcd_byte(line, self.LCD_CMD)
      for i in range(self.LCD_WIDTH):
         self.lcd_byte(ord(message[i]),self.LCD_CHR)
         
   def lcd_cursor(self, row, col):
        if row == 0:
            col |= 0x80
        elif row == 1:
            col |= 0xC0
        self.lcd_byte(col, False)
