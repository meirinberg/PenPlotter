"""!
@file main.py
@author 
@date    
"""

import gc
#import pyb
import cotask
import task_share
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
from array import array
import os

global args, commands


# datay = []

# MAY HAVE TO CHANGE MODE BACK TO 0

def File_Draw():
    """
    Reads loaded drawing data and converts to usable form
    """
    motor_radial = StepperMotor(1, 0b00010000, 1000, 1500, False)
    motor_radial.setXActualToZero()
    motor_theta = StepperMotor(2, 0b00000001, 1000, 1023, False)
    motor_theta.setXActualToZero()
    light.on()
    line1 = "Print Started"
    line2 = ""
    display.lcd_string(line1, display.LCD_LINE_1)
    display.lcd_string(line2, display.LCD_LINE_2)
    while True:
        if state.get() == 3:                
            if mode.get() == 0:
                
                bool1 = motor_theta.travelToPosition(datax[ii.get()],10)
                bool2 = motor_radial.travelToPosition(datay[ii.get()],10)
                if (bool1 and bool2):
                    pen.down()
                    mode.put(1)
                    ii.put(1)
            elif mode.get() == 1:
#                 print(mode.get())
                bool1 = motor_theta.travelToPosition(datax[ii.get()],0)
                bool2 = motor_radial.travelToPosition(datay[ii.get()],0)
                
                if (stick.readSwitch()):
                    print('Cancellation Activated')
                    state.put(0)
                    light.on()
                    line1 = "Print Cancelled"
                    line2 = ""
                    display.lcd_string(line1, display.LCD_LINE_1)
                    display.lcd_string(line2, display.LCD_LINE_2)
                    UIMode = True
                    motor_theta.travelToPosition(0,0)
                    motor_radial.travelToPosition(0,0)
                
                if (bool1 and bool2):
                    print("Got to point")
                    
                    ii.put(ii.get()+1)
                    #LCD commands
                    
                    if ii.get() == len(datax):
                        ii.put(0)
                        state.put(0)
                        pen.up()
                        
                        #LCD commands
                        light.on()
                        line1 = "Print Completed"
                        line2 = ""
                        display.lcd_string(line1, display.LCD_LINE_1)
                        display.lcd_string(line2, display.LCD_LINE_2)
                        UIMode = True
                        motor_theta.travelToPosition(0,0)
                        motor_radial.travelToPosition(0,0)
                        light.off()
                    else:
                        yield None
                else:
                    yield None
        else:
            yield None

#         if ii.get() == (len(datax)-1):
#             state.put(4)
#             ii.put(0)
#         if state.get() == 3:  
#             if theta.num_in() < 20:
#                 theta.put(datax[ii.get()])
#                 ii.put(ii.get()+1)
#                 theta.put(datay[ii.get()])
#                 
#             else:
#                 yield None
#         else:
#             yield None
        

def Free_Draw():
    """
    Runs Stepper Motors in Free Draw Mode
    """
    motor_radial = StepperMotor(1, 0b00010000, 1023, 1023, True)
    motor_theta = StepperMotor(2, 0b00000001, 1000, 1023, True)
   
    motor_radial.setVelocity(3000,5000)
    motor_theta.setVelocity(3000,5000)
   
    radius = 0  # This should correspond to where the robot starts
    theta = 0   # This should correspond to where the robot starts
    
    while True:
        # Show everything currently in the queue and the value in the share
        if state.get() == 4:
            
    #       THETA_SCALAR = 35 
    #       RADIUS_SCALAR = 35
    # 
    #       radius = radius + int((stick.readY()-2048) / 4096 * THETA_SCALAR)
    #       theta = theta + int((stick.readX()-2048) / 4096 * RADIUS_SCALAR)
    
          radial_speed = 1000
          theta_speed = 500
          print(stick.readY(), stick.readX())
          if stick.readX() > 2248:
              radius = -1*radial_speed
          elif stick.readX() < 1848:
              radius = radial_speed
          else:
              radius = 0
              
          targetY = motor_radial.convertIntToBytes(radius)
          
          if stick.readY() > 2248:
              theta = -1*theta_speed
          elif stick.readY() < 1848:
              theta = theta_speed
          else:
              theta = 0
              
          targetX = motor_theta.convertIntToBytes(theta)
          

          radialTargetByteSet = bytearray([motor_radial.VTARGET_ADDR, 
                                      targetY[0],
                                      targetY[1],
                                      targetY[2]])
          
          thetaTargetByteSet = bytearray([motor_theta.VTARGET_ADDR, 
                                      targetX[0],
                                      targetX[1],
                                      targetX[2]]) 
          
          line1 = "Theta: " + str(theta)
          line2 = "Radius: " + str(radius)
          display.lcd_string(line1, display.LCD_LINE_1)
          display.lcd_string(line2, display.LCD_LINE_2)
          
          motor_radial.sendByteSet(radialTargetByteSet)
          motor_theta.sendByteSet(thetaTargetByteSet)
          #print(motor_radial.readPosition())
    #       print(motor_radial.travelToPosition(radius))
    #       print(motor_theta.travelToPosition(theta))
    
          # If the joystick button is pressed, turn the light on and toggle the pen up/down.
          if (stick.readSwitch()):
             light.on()
             # Toggle Pen Up/Down
             if pen.getValue():
                pen.down()
             else:
                pen.up()
          else:
             light.off()
        else:
            yield None


def Calc_HPGL():
    
    '''
    Calculates all interpolated values from HPGL
    '''
    display.lcd_string('Parsing HPGL', display.LCD_LINE_1)
    display.lcd_string(">>>>>>>>>>>>>>>>", display.LCD_LINE_2)
    global datax
    global datay
    datax = []
    datay = []
    while True:
        
        if state.get() == 2:
            # Show everything currently in the queue and the value in the share
            dpi = 1016
            args = []
            commands = []
            alpha = 3.14159*25.4
            # if filename:
            filename = str(selected)
            print(type(selected))
            with open(selected)  as f:
                # Store all the lines of the file into the lines array.
                lines = f.readlines()
                # Split this string into an array of elements.

            for i in range(0, len(lines)):
                formattedLine = lines[i]
                # Remove extraneous newline characters.
                if '\n' in formattedLine:
                    formattedLine = formattedLine.replace('\n', '')
                    # Remove extraneous spaces.
                if ' ' in formattedLine:
                    formattedLine = formattedLine.replace(' ', '')
                    # Split each line by its semicolons to identify the commands.
                splitBySemicolon = formattedLine.split(';')
                for i in range(0, len(splitBySemicolon)):
                    commands.append(splitBySemicolon[i][:2])
                    args.append(splitBySemicolon[i][2:].split(','))


            # print("Commands", commands)
            # print("Args", args)

#             datax = []
#             datay = []
#             prepx = arr.array('d',[0,0])
#             prepy = arr.array('d',[0,0])

            for i in range(0,len(args[6]),2):
                datax.append((int(args[6][i]))/dpi)
                datay.append((int(args[6][i+1]))/dpi)

#             prepx[0] = int(args[3][0])/dpi
#             prepx[1] = int(args[5][0])/dpi
#             prepy[0] = int(args[3][1])/dpi
#             prepy[1] = int(args[5][1])/dpi

            Xprocess = []
            Yprocess = []
            Xprocess.append(datax[0])
            Yprocess.append(datay[0])

            s = 0
            for i in range(1,len(datax)):
                mag = math.sqrt((datax[i]-datax[i-1])**2 + (datay[i]-datay[i-1])**2)
                go = False
                if mag > 2:
                    split = 20
                    go = True
                elif mag > 1.5:
                    split = 15
                    go = True
                elif mag > 0.5:
                    split = 10
                    go = True
                if go:
                    xadd,yadd = interp([datax[i-1],datax[i]],[datay[i-1],datay[i]],split)
                    for j in range(0,len(xadd-1)):
                        Xprocess.append(xadd[j])
                        Yprocess.append(yadd[j])
                        
                    s += split
                    go = False
                else:
                    Xprocess.append(datax[i])
                    Yprocess.append(datay[i])
                    s+=1

            datax = Xprocess
            datay = Yprocess

            theta = [0.1,0.1]
            guess = [math.atan(datay[0]/datax[0]),math.sqrt(datax[0]**2 + datay[0]**2)]

            for t in range(len(datax)):
                # Calculates angular position output using Newton Raphson function for each
                # theoretical data point.
                X = [datax[t],datay[t]]
                temp = NewtonRaphson(lambda theta: g(X, theta), dg_dtheta, guess, .01)
                temp[0] = abs(temp[0]%(math.pi))
                temp[1] = abs(temp[1])
                datax[t] = int((temp[0])*NEMA.get() / (2*math.pi))
                datay[t] = int((temp[1])*OTHERSTEP.get()*100 / (2*math.pi))
                
                guess=temp
                
            print(datax)
            state.put(3)
            
        else:
            yield None
def UI():
    global selected
    '''
    Primarily used for startup and sending commands to LCD
    '''
    while True:
        if state.get() == 0:
            i = 0
            line1 = filenames[i]
            line2 = filenames[i+1]
            state.put(1)
        elif state.get() == 1:
            if (stick.readX() > 3048) and i < len(filenames)-1 and i >= 0:
                #print("DOWN")
                i+=1
            elif (stick.readX() < 1048) and i <= len(filenames)-1 and i > 0:
                #print("UP")
                i-=1
        
            if i == len(filenames)-1 and (i % 2 == 0):
                line1 = filenames[i]
                if (i+1) < len(filenames):
                    line2 = filenames[i+1]
                else:
                    line2 = ""
            elif i <= len(filenames)-1 and not (i % 2 == 1):
                #print("HERE")
                if (i+1) < len(filenames):
                    #print("HERE2")
                    line1 = filenames[i]
                    line2 = filenames[i+1]
            elif (i % 2 == 1):
                line1 = filenames[i-1]
                line2 = filenames[i]
            
        
            if (i % 2 == 0):
                display.lcd_string(">" + line1, display.LCD_LINE_1)
                display.lcd_string(line2, display.LCD_LINE_2)
            else:
                display.lcd_string(line1, display.LCD_LINE_1)
                display.lcd_string(">" + line2, display.LCD_LINE_2)
        
            if (stick.readSwitch()):
                light.on()
                UIMode = False
                selected = filenames[i]
                print("Selected:", selected)
                if selected == "Free Draw":
                    state.put(4)
                else:
                    state.put(2)
            else:
                light.off()
            
            yield None
        else:
            yield None

# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    print ('Welcome to Pen Plotter, use joystick to select mode')
    '''
    Creating Shares, Queues, and Tasks
    '''
    # Create a share and a queue to test function and diagnostic printouts
    state = task_share.Share ('h', thread_protect = False, name = "State")
    state.put(0)
    ii = task_share.Share ('h', thread_protect = False, name = "Index ii")
    ii.put(0)
    
    mode = task_share.Share ('h', thread_protect = False, name = "Mode")
    mode.put(0)
    
    NEMA = task_share.Share ('h', thread_protect = False, name = "NEMA Rotation")
    NEMA.put(1600)
    
    OTHERSTEP = task_share.Share ('h', thread_protect = False, name = "OTHER STEPPER Rotation")
    OTHERSTEP.put(385)
    
    filetype = task_share.Share ('h', thread_protect = False, name = "File Type")
    
    theta = task_share.Queue ('L', 20, thread_protect = False, overwrite = False,
                           name = "HPGL Output")
#     thetay = task_share.Queue ('L', 10, thread_protect = False, overwrite = False,
#                            name = "HPGL Y Output")
    
    File_Draw = cotask.Task (File_Draw, name = 'File_Draw', priority = 1, 
                         period = 100, profile = True, trace = False)
    Free_Draw = cotask.Task (Free_Draw, name = 'Free_Draw', priority = 2, 
                         period = 100, profile = True, trace = False)
    Calc_HPGL = cotask.Task (Calc_HPGL, name = 'Calc_HPGL', priority = 3, 
                         period = 1000, profile = True, trace = False)
    UI = cotask.Task (UI, name = 'UI', priority = 4, 
                         period = 100, profile = True, trace = False)
#     Run = cotask.Task (Run, name = 'Run', priority = 2, 
#                          period = 1000, profile = True, trace = False)
    
    
    
    cotask.task_list.append(Free_Draw)
    cotask.task_list.append(Calc_HPGL)
    cotask.task_list.append(File_Draw)
    cotask.task_list.append(UI)
    
    
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()

    # Run the scheduler with the chosen scheduling algorithm. Quit if any 
    # character is received through the serial port
    vcp = pyb.USB_VCP ()
    
    # Show everything currently in the queue and the value in the share
    selected = ""
    filenames = ["Free Draw"]
    for x in os.listdir():
        if x.endswith(".hpgl") and not x.startswith('._'):
            # Prints only text file present in My Folder
            filenames.append(x)
    
    
    '''
----------------------------------All Classes Specified Here-------------------------------------
    '''
    
    class LED: 
    
       def __init__(self):
          # Joystick Pin Connections
          self.LEDPIN = Pin(Pin.cpu.C4, mode=Pin.OUT)
    
       def on(self):
          self.LEDPIN.high()
    
       def off(self):
          self.LEDPIN.low()
    
       def getValue(self):
          return not self.LEDPIN.value()  # If this returns true, the LED is ON. Otherwise, the LED is OFF.
    
    class Solenoid: 
    
       def __init__(self):
          # Joystick Pin Connections
          self.PENPIN = Pin(Pin.cpu.A1, mode=Pin.OUT)
    
       def down(self):
          self.PENPIN.high()
    
       def up(self):
          self.PENPIN.low()
    
       def getValue(self):
          return not self.PENPIN.value()  # If this returns true, the pen is down. Otherwise, the pen is up.
    
    
    class Joystick: 
    
       def __init__(self):
          # Joystick Pin Connections
          self.XPIN = Pin(Pin.cpu.A5, mode=Pin.IN)
          self.YPIN = Pin(Pin.cpu.A6, mode=Pin.IN)
          self.BUTTONPIN = Pin(Pin.cpu.C5, mode=Pin.IN, pull=pyb.Pin.PULL_UP)
          self.x = pyb.ADC(self.XPIN)    # create an analog object from a pin
          self.y = pyb.ADC(self.YPIN)    # create an analog object from a pin
    
       def readX(self):
          return self.x.read()
    
       def readY(self):
          return self.y.read()
    
       def readSwitch(self):
          return not self.BUTTONPIN.value() # If this returns true, the switch is pressed down. Otherwise, the switch is not pressed.
    
    
    class LCD: 
       # Define some device constants
       LCD_WIDTH = 16    # Maximum characters per line
       LCD_CHR = True
       LCD_CMD = False
    
       LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
       LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
    
       # Timing Constants
       E_PULSE = 0.00005
       E_DELAY = 0.00005
    
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
    
    
    class StepperMotor: 
    
       NEMA_FULL_ROTATION = 1600
       OTHER_FULL_ROTATION = 385
    
       PMUL_PDIV_ADDR =        0b00010010
       TYPE_VER_ADDR =         0b01110011
       ENABLE_ADDR =           0b01101000 
       VMIN_ADDR =             0b00000100
       VMAX_ADDR =             0b00000110
       PULSE_RAMP_ADDR =       0b00011000 
       AMAX_ADDR =             0b00001100
       MODE_ADDR =             0b01110010
       XACTUAL_ADDR =          0b00000010
       XTARGET_ADDR =          0b00000000
       VTARGET_ADDR =          0b00001000
       LIMIT_SWITCHES_ADDR =   0b00010100
       
       #accelerationValue = 0b00000001
    
       def __init__(self, chipSelect, acceleration, vMin, vMax, vMode):
          # Polarity = 1, Phase = 1
          # Pin Set-Up
          self.vMode = vMode
          self.accelerationValue = acceleration
          self.vMin = vMin
          self.vMax = vMax
          self.chipSelect = chipSelect
          self.PC6 = Pin("C6", mode = Pin.OUT_PP)
          self.tim = pyb.Timer(3, period = 3, prescaler = 0)
          self.clk = self.tim.channel(1, pin = self.PC6, mode = pyb.Timer.PWM, pulse_width = 2) #Channel 1 or 2?
          self.nCS1 = Pin("C0", mode = Pin.OUT, value = 1)
          self.nCS2 = Pin("B0", mode = Pin.OUT, value = 1)
          self.en1 = Pin("C3", mode = Pin.OUT, value = 0)
          self.en2 = Pin("C2", mode = Pin.OUT, value = 0)
          # SPI Set-Up
          self.spi = SPI(2, SPI.CONTROLLER, baudrate=1000000, polarity=1, phase = 1, firstbit = SPI.MSB) 
    
          # Calculate the correct pdiv from an acceleration value.
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
           
          # Send the byte sets:
          #Motor.sendByteSet(testByteSet)
          
          if vMode:
             self.sendByteSet(enableByteSet)
             self.setVelocity(self.vMin,self.vMax)
             self.sendByteSet(pulseAndRampDivByteSet)
             self.sendByteSet(aMaxByteSet)
             self.sendByteSet(calculated_Pdiv)
             self.sendByteSet(vrampModeByteSet)
             self.sendByteSet(vTargetByteSet)
             print(self.sendByteSet(readModeByteSet))
             
          else:
             self.sendByteSet(enableByteSet)
             self.setVelocity(self.vMin,self.vMax)
             self.sendByteSet(pulseAndRampDivByteSet)
             self.sendByteSet(aMaxByteSet)
             self.sendByteSet(calculated_Pdiv)
             #self.sendByteSet(vrampModeByteSet)
             #self.sendByteSet(vTargetByteSet)
             self.sendByteSet(xrampModeByteSet)
             self.setXActualToZero()
    
       def convertIntToBytes(self,value):
          firstByte = value & 0xFF
          secondByte = value >> 8 & 0xFF
          thirdByte = value >> 16 & 0xFF
          return (thirdByte,secondByte,firstByte)
        
       def convertBytesToInt(self,value):
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
          temp1 = self.convertIntToBytes(vmin)
          temp2 = self.convertIntToBytes(vmax)
          vminByteSet = bytearray([self.VMIN_ADDR,temp1[0],temp1[1],temp1[2]])
          vmaxByteSet = bytearray([self.VMAX_ADDR,temp2[0],temp2[1],temp2[2]]) 
          self.sendByteSet(vminByteSet)
          self.sendByteSet(vmaxByteSet)
            
       def sendByteSet(self, byteSet):
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
          pd = range(14)
          pm = range(128,256)
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
                
          #print ("PMUL: ", pmul, "PDIV: ", pdiv)
          return pmul, pdiv
    
       def sendPosition(self, position):
          targetValue = self.convertIntToBytes(position)
          target = bytearray([self.XTARGET_ADDR,
                               targetValue[0],
                               targetValue[1],
                               targetValue[2]])
          self.sendByteSet(target)
        
       def readPosition(self):
          result = bytearray([0b00000011,
                               0,
                               0,
                               0])
          currentPosition = self.sendByteSet(result)
          answer = self.convertBytesToInt(currentPosition)
          return answer
        
       def travelToPosition(self, position,thresh):
           self.sendPosition(position)
           currentPosition = self.readPosition()
           print("Sent:", position, "Read:", currentPosition)
           if abs(currentPosition - position) <= thresh:
               return True
           else:
               return False
           # Return True if motor made it to the target position, False otherwise.
           
           #time.sleep(0.5)
#            return currentPosition == position
        
       def setXActualToZero(self):
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
    
    
    '''
    ---------------------Defining Image Processing Functions-----------------------
    '''
    def interp(xpoint,ypoint,num):    
        ix = np.linspace(xpoint[0],xpoint[1],num)
        iy = np.linspace(ypoint[0],ypoint[1],num)
        return ix, iy


    def g(x, theta):
        # Computes difference between expected and calculated pen plotter angular position
        #print("Theta",theta)
        f1 = theta[1]*math.cos(theta[0])
        f2 = theta[1]*math.sin(theta[0])
        #print("X",x)
        Theta = np.array([f1,f2])
        return x-Theta
        
    def dg_dtheta(theta):
        # Computes Jacobian matrix for inverse kinematics
        df1dt1 = -theta[1]*math.sin(theta[0])# /alpha
        df1dt2 = math.cos(theta[0])# /alpha
        df2dt1 = theta[1]*math.cos(theta[0])# /alpha
        df2dt2 = math.sin(theta[0])# /alpha
        jacob = np.array([[-1*df1dt1, -1*df1dt2],[-1*df2dt1,-1*df2dt2]])
        return jacob

    def NewtonRaphson(fcn, jacobian, guess, thresh):
        # This is called repeatedly to iterate on the correct angular position to produce the desired
        # pen plotter position.
        steps = 1

        while (abs(fcn(guess)[0]) > thresh or abs(fcn(guess)[1]) > thresh):
            
            math = guess - np.dot(np.linalg.inv(jacobian(guess)),fcn(guess))

            guess = math
            steps += 1
        return guess

    
    # Initialize objects from each class.
    light = LED()
    pen = Solenoid()
    stick = Joystick()
    display = LCD()
    
    while not vcp.any ():
        cotask.task_list.pri_sched()

    # Empty the comm port buffer of the character(s) just pressed
    vcp.read()

    # Print a table of task data and a table of shared information data
#     print ('\n' + str (cotask.task_list))
#     print (task_share.show_all ())
#     print (task1.get_trace ())
#     print ('\r\n')

