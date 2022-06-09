import gc
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array


class StepperMotor: 
   '''
   This class directly interacts with the TMC 4210 and TMC 2208 drivers
   to drive the stepper motors. Registers, system constants, byte
   conversions, and functions to read and write motor data are specified.
   See datasheets for TMC 4210 and TMC 2208 for more details.
   '''
   NEMA_FULL_ROTATION = 1600
   OTHER_FULL_ROTATION = 385

   PMUL_PDIV_ADDR =        0b00010010 # PMUL and PDIV register 
   TYPE_VER_ADDR =         0b01110011 # Register with defined address to verify correclty reading
   ENABLE_ADDR =           0b01101000 # Driver enabling register 
   VMIN_ADDR =             0b00000100 # Minimum motor velocity register 
   VMAX_ADDR =             0b00000110 # Maximum motor velocity register
   PULSE_RAMP_ADDR =       0b00011000 # Pulse and Ramp generator register
   AMAX_ADDR =             0b00001100 # Maximum acceleration registser
   MODE_ADDR =             0b00010100 # Driver operating mode register
   XACTUAL_ADDR =          0b00000010 # Motor actual angular position register
   XTARGET_ADDR =          0b00000000 # Motor target angular position register 
   VTARGET_ADDR =          0b00001000 # Motor target angular velocity register
   LIMIT_SWITCHES_ADDR =   0b00010100 # Limit switch register

   def __init__(self, chipSelect, acceleration, vMin, vMax, vMode):
       # Specifies chipSelect, motor max acceleration, motor minimum velocity,
       # motor maximum velocity, and whether operation is in velocity (True)
       # or ramp mode (False)
      self.vMode = vMode
      self.accelerationValue = acceleration
      self.vMin = vMin
      self.vMax = vMax
      self.chipSelect = chipSelect
      
      # Pin Set-Up
      # External clock signal created for TMC 4210 to perform step sequencing
      # Nucleo has 80MHz. Using period = 3, prescaler = 0, to create 20MHz frequency
      self.PC6 = Pin("C6", mode = Pin.OUT_PP) # Pin for clock signal
      self.tim = pyb.Timer(3, period = 3, prescaler = 0)
      self.clk = self.tim.channel(1, pin = self.PC6, mode = pyb.Timer.PWM, pulse_width = 2) 
      
      # Chip select pins, active low
      self.nCS1 = Pin("C0", mode = Pin.OUT, value = 1) 
      self.nCS2 = Pin("B0", mode = Pin.OUT, value = 1)
      self.en1 = Pin("C3", mode = Pin.OUT, value = 0)
      self.en2 = Pin("C2", mode = Pin.OUT, value = 0)
      
      # SPI Set-Up
      self.spi = SPI(2, SPI.CONTROLLER, baudrate=1000000, polarity=1, phase = 1, firstbit = SPI.MSB) 

      # Calculate the correct pmul, pdiv from an acceleration value.
      pmul, pdiv = self.set_accel(self.accelerationValue)
       
      # Populate the byte sets with the appropriate data for each address.
      calculated_Pdiv = bytearray([self.PMUL_PDIV_ADDR,
                                   0b00000000,
                                   pmul,
                                   pdiv & 0b00001111])
       
      testByteSet = bytearray([self.TYPE_VER_ADDR,
                               0b00000000,
                               0b00000000,
                               0b00000000])
       
      enableByteSet = bytearray([self.ENABLE_ADDR,
                                 0b00000000,
                                 0b00000000,
                                 0b00100000]) 
       
      pulseAndRampDivByteSet = bytearray([self.PULSE_RAMP_ADDR,
                                          0b00000000,
                                          0b01110111, # 0b01110111 for microsteps
                                          0b00000000]) 
       
      aMaxByteSet = bytearray([self.AMAX_ADDR,
                               0b00000000,
                               0b00000000,
                               self.accelerationValue]) 
       
      vrampModeByteSet = bytearray([self.MODE_ADDR,
                                    0b00000000,
                                    0b00000000,
                                    0b00000010])
       
      xrampModeByteSet = bytearray([self.MODE_ADDR,
                                    0b00000000,
                                    0b00000000,
                                    0b00000000])
       
      xTargetByteSet = bytearray([self.XTARGET_ADDR,
                                  0b00000000,
                                  0b00000000,
                                  0b10011011])

      zeroTarget = bytearray([self.XTARGET_ADDR,
                              0b00000000,
                              0b00000000,
                              0b00000000]) 
       
      vTargetByteSet = bytearray([self.VTARGET_ADDR, 
                                  0b00000000,
                                  0b00000000,
                                  0b00000000]) 
       
      limitswitches = bytearray([self.LIMIT_SWITCHES_ADDR, 
                                 0b00000000,
                                 0b00000011,
                                 0b00010000])
      
      readModeByteSet = bytearray([0b01110011,
                                    0b00000000,
                                    0b00000000,
                                    0b00000000])
       
     
      if vMode:
          # Order to send commands and run motor is important and must
          # be followed in this order for activating velocity mode.
         self.sendByteSet(enableByteSet)
         self.setVelocity(self.vMin,self.vMax)
         self.sendByteSet(pulseAndRampDivByteSet)
         self.sendByteSet(aMaxByteSet)
         self.sendByteSet(calculated_Pdiv)
         self.sendByteSet(vrampModeByteSet)
         self.sendByteSet(vTargetByteSet)
         print(self.sendByteSet(readModeByteSet))
         
      else:
          # Order to send commands and run motor is important and must
          # be followed in this order for activating ramp mode.
         self.sendByteSet(enableByteSet)
         self.setVelocity(self.vMin,self.vMax)
         self.sendByteSet(pulseAndRampDivByteSet)
         self.sendByteSet(aMaxByteSet)
         self.sendByteSet(calculated_Pdiv)
         self.sendByteSet(xrampModeByteSet)
         self.setXActualToZero()

   def convertIntToBytes(self,value):
       # Converts integers into a 3 bytes array
       firstByte = value & 0xFF
       secondByte = value >> 8 & 0xFF
       thirdByte = value >> 16 & 0xFF
       return (thirdByte,secondByte,firstByte)
    
   def convertBytesToInt(self,value):
       # Convert byte array into integer
       r = 0;
       bitmask = 0xff;
       b1 = value[1]
       b2 = value[2]
       b3 = value[3]

       if ((b1 & 0x80) != 0):
           r |= bitmask << 24
          
       r |= b1 << 16
       r |= b2 << 8
       r |= b3
       return r 

   def setVelocity(self, vmin, vmax):
       # Set minimum and maximum velocity of motors
      temp1 = self.convertIntToBytes(vmin)
      temp2 = self.convertIntToBytes(vmax)
      vminByteSet = bytearray([self.VMIN_ADDR,temp1[0],temp1[1],temp1[2]])
      vmaxByteSet = bytearray([self.VMAX_ADDR,temp2[0],temp2[1],temp2[2]]) 
      self.sendByteSet(vminByteSet)
      self.sendByteSet(vmaxByteSet)
        
   def sendByteSet(self, byteSet):
       # Sends byte arrays to driver
         if self.chipSelect == 1:
            self.nCS1.low()
            data = self.spi.send_recv(byteSet)
            #print(data)
            self.nCS1.high()
            return data
         elif self.chipSelect == 2:
            self.nCS2.low()
            data = self.spi.send_recv(byteSet)
            #print(data)
            self.nCS2.high()
            return data
         else:
            return None

    
   def set_accel(self, max_accel):
       # Given a maximum accleration, the function will calculate the
       # proper PMUL and PDIV integer vales to be sent the drivers
      pd = range(14) # Possible PDIV values
      pm = range(128,256) # Possible PMUL values
      pmul = 128
      valid = 0
      while pmul<len(pm)+128:
         pdiv = 0
         while pdiv<len(pd):
            q = (pmul/max_accel)*2**(4-pdiv)
            if q>0.095 and q<1:
               valid = 1
               break
            else:
               pdiv += 1
         if valid == 1:
            break
         else:
            pmul += 1
            
      print ("PMUL: ", pmul, "PDIV: ", pdiv)
      return pmul, pdiv

   def sendPosition(self, position):
       # Sends desired position to motor
      targetValue = self.convertIntToBytes(position)
      target = bytearray([self.XTARGET_ADDR,
                           targetValue[0],
                           targetValue[1],
                           targetValue[2]])
      self.sendByteSet(target)
    
   def readPosition(self):
       # Reads current angular position of motor
      result = bytearray([0b00000011,
                           0,
                           0,
                           0])
      currentPosition = self.sendByteSet(result)
      answer = self.convertBytesToInt(currentPosition)
      return answer
    
   def travelToPosition(self, position,thresh):
       # Sends desired position to motor, returns True or False
       # when motor has actually reached sent position
       self.sendPosition(position)
       currentPosition = self.readPosition()
       print("Sent:", position, "Read:", currentPosition)
       if abs(currentPosition - position) <= thresh:
           return True
       else:
           return False
       # Return True if motor made it to the target position, False otherwise.
       
   
   def setXActualToZero(self):
       # Sets current motor position to zero. Used for homing operations.
      vModeByteSet = bytearray([self.MODE_ADDR,
                                    0b00000000,
                                    0b00000000,
                                    0b00000010])
      xrampModeByteSet = bytearray([self.MODE_ADDR,
                                    0b00000000,
                                    0b00000000,
                                    0b00000000])
      xActualByteSet = bytearray([self.XACTUAL_ADDR,
                                  0b00000000,
                                  0b00000000,
                                  0b00000000])
      self.sendByteSet(vModeByteSet)
      self.sendByteSet(xActualByteSet)
      self.sendByteSet(xrampModeByteSet)
