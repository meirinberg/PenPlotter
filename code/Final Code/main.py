"""!
@file demoDay_Final.py
@author Ryan Dean, Mike Eirenberg, Ben Bradley
@date 6/9/2022
"""

import gc
import cotask
import task_share
from ulab import numpy as np
import math
import pyb
from pyb import Pin, Timer, ADC, ExtInt, SPI
import time
import array
import os
import stepperMotor
import lcd
import solenoid
import led
import joystick
global args, commands

def File_Draw():
    
    """
    This task is responsible for taking processed HPGL data and
    sending it to the stepper motors. The radial motor is set with
    a much higher acceleration than the theta motor because
    the theta motor has much more interia to move and therefore
    will need more time to accelerate and decelerate the pen
    plotter assembly.
    """
    
    # Initializes motors in Ramp Mode
    motor_radial = stepperMotor.StepperMotor(1, 0b00001000, 1000, 1500, False)
    motor_theta = stepperMotor.StepperMotor(2, 0b00000001, 1000, 1023, False)
    
    # Sets starting position as homing prosition when done printing
    motor_radial.setXActualToZero()
    motor_theta.setXActualToZero()
    
    # Turns LED light on
    light.on()
    
    # LCD display commands
    line1 = "Print Started"
    line2 = ""
    display.lcd_string(line1, display.LCD_LINE_1)
    display.lcd_string(line2, display.LCD_LINE_2)
    
    while True:
        if state.get() == 3:                
            # This ensures the motor reaches the first drawing point before the
            # plotting pen goes down
            if mode.get() == 0:
                # Returned value is True when position is reached, False otherwise
                bool1 = motor_theta.travelToPosition(int(datax[ii.get()]),10)
                bool2 = motor_radial.travelToPosition(int(datay[ii.get()]),10)
                if (bool1 and bool2):
                    pen.down() # Controls plotting pen
                    mode.put(1)
                    ii.put(1) # Used for data indexing
            
            elif mode.get() == 1:
                # Iterates through Theta (datax) and Radial (datay),
                # sending positions to motors
                bool1 = motor_theta.travelToPosition(int(datax[ii.get()]),0)
                bool2 = motor_radial.travelToPosition(int(datay[ii.get()]),0)
                
                
                if not CancelButton.value():
                    # Print cancellation when blue button on STM 32 pressed
                    print('Free Draw Cancelled')
                    print('Cancellation Activated')
                    state.put(0)
                    light.on()
                    pen.up()
                    
                    # LCD commands
                    line1 = "Print Cancelled"
                    line2 = ""
                    display.lcd_string(line1, display.LCD_LINE_1)
                    display.lcd_string(line2, display.LCD_LINE_2)
                    
                    # Tells motors to return to homing position
                    motor_theta.travelToPosition(0,0)
                    motor_radial.travelToPosition(0,0)
                    mode.put(0)
                
                
                if (bool1 and bool2):
                    # LCD commands to update LCD each time the motor reaches
                    # its desired position
                    p = ii.get()
                    print("Got to point")
                    
                    # LCD commands
                    line1 = "Printing..."
                    line2 = "Point " + str(p) + " of " + str(len(datax))
                    display.lcd_string(line1, display.LCD_LINE_1)
                    display.lcd_string(line2, display.LCD_LINE_2)
                    ii.put(ii.get()+1)
                    
                    # Checks if end of received data has been reached
                    if ii.get() == len(datax):
                        ii.put(0)
                        state.put(0)
                        
                        # Brings pen back up
                        pen.up()
                        
                        # LCD commands                        
                        light.on()
                        line1 = "Print Completed"
                        line2 = ""
                        display.lcd_string(line1, display.LCD_LINE_1)
                        display.lcd_string(line2, display.LCD_LINE_2)
#                         UIMode = True

                        # Tells motors to return to homing position
                        motor_theta.travelToPosition(0,0)
                        motor_radial.travelToPosition(0,0)
                        light.off() # Turns off LED
                        mode.put(0)
                    else:
                        yield None
                else:
                    yield None
        else:
            yield None

def Free_Draw():
    """
    This task is responsible for the 'free draw' feature, allowing
    the user to draw anything they want using the built-in joystick.
    It is still in polar coordinates, with left and right joystick
    motions controlling the Theta motor, and up and down directions
    controlling the Radial motor.
    """
    
    # Initializes motors in Velocity Mode 
    motor_radial = stepperMotor.StepperMotor(1, 0b00010000, 3000, 5000, True)
    motor_theta = stepperMotor.StepperMotor(2, 0b00000001, 3000, 5000, True)
    
    
    while True:
        if state.get() == 4:
           
           # Reads ADC data from joystick X (Theta) and Y (Radial) directions
           # Offsets readings to produce centered ADC output when there
           # is no input from user
           radial_speed = -1*(stick.readX()-2048)
           theta_speed = -1*(stick.readY()-2048)
           
           # These conditionals create a tolerance for when the
           # motors will activate under a joystick input since
           # joystick output is erratic
           if abs(radial_speed) > 200:
               targetY = motor_radial.convertIntToBytes(radial_speed)
           else:
               radial_speed = 0
               targetY = motor_radial.convertIntToBytes(0)
           if abs(theta_speed) > 200:
               targetX = motor_theta.convertIntToBytes(theta_speed)
           else:
               theta_speed = 0
               targetX = motor_theta.convertIntToBytes(0)
  
            
           # Creates speed input byte array for motor velocity register 
           radialTargetByteSet = bytearray([motor_radial.VTARGET_ADDR, 
                              targetY[0],
                              targetY[1],
                              targetY[2]])
  
           thetaTargetByteSet = bytearray([motor_theta.VTARGET_ADDR, 
                              targetX[0],
                              targetX[1],
                              targetX[2]]) 
          
           # LCD commands to display Theta and Radius joystick values
           line1 = "Theta: " + str(theta_speed)
           line2 = "Radius: " + str(radial_speed)
           display.lcd_string(line1, display.LCD_LINE_1)
           display.lcd_string(line2, display.LCD_LINE_2)
  
           # Sends joystick velocity commands to motors
           motor_radial.sendByteSet(radialTargetByteSet)
           motor_theta.sendByteSet(thetaTargetByteSet)
             
           # Cancels free draw mode to return to UI state
           if not CancelButton.value():
                    print('Free Draw Cancelled')
#                 if (stick.readSwitch()):
                    print('Cancellation Activated')
                    state.put(0)
                    light.on()
                    pen.up()
                    line1 = "Print Cancelled"
                    line2 = ""
                    display.lcd_string(line1, display.LCD_LINE_1)
                    display.lcd_string(line2, display.LCD_LINE_2)
                    
           # Reads if joystick has been pressed, putting pen down or up for drawing
           if (stick.readSwitch()):
               light.on()
               if pen.getValue():
                   pen.down()
               else:
                   pen.up()
               yield None
                   
           else:
               yield None
               
        else:
            yield None


def Calc_HPGL():
    
    '''
    This task takes Inkscape HPGL data and converts it into
    angular data that can be interpretted by the motors based on the
    polar coordinate geometry of the system. Data must be interpolated
    between data points with large distances to ensure both motors
    arrive at their desired positions. Data is appended to global
    variable lists to be used by other tasks that send commands
    directly to the motors.
    '''
    display.lcd_string('Parsing HPGL', display.LCD_LINE_1)
    display.lcd_string(">>>>>>>>>>>>>>>>", display.LCD_LINE_2)
    
    # Global variable creation as arrays
    global datax
    global datay
    datax = array.array('d')
    datay = array.array('d')
    
    while True:
        
        if state.get() == 2:
            
            dpi = 1016 # Dots per Inch taken from Inkscape
            args = [] # Will contain numerical data points
            commands = [] # Will contain pen commands common in HPGL formatting
            alpha = 3.14159*25.4 # Constant based on hand calculations

            # Reads selected HPGL file received from UI task
            filename = str(selected)
            print(type(selected))
            with open(selected)  as f:
                # Store all the lines of the file into the lines array.
                lines = f.readlines()
    
            # For each line in HPGL data
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
                
                # Add commands and HPGL data to commands and args respectively
                for i in range(0, len(splitBySemicolon)):
                    commands.append(splitBySemicolon[i][:2])
                    args.append(splitBySemicolon[i][2:].split(','))

            # Assigns relevant HPGL data
            for i in range(0,len(args[6]),2):
                datax.append((int(args[6][i]))/dpi)
                datay.append((int(args[6][i+1]))/dpi)

            # Will contain all original and interpolated data
            Xprocess = array.array('d')
            Yprocess = array.array('d')
            Xprocess.append(datax[0])
            Yprocess.append(datay[0])

            # Defines extent of interpolation based on distance threshold
            # between data points
            s = 0
            for i in range(1,len(datax)):
                mag = math.sqrt((datax[i]-datax[i-1])**2 + (datay[i]-datay[i-1])**2)
                go = False
                if mag > 2:
                    split = 50
                    go = True
                elif mag > 1.5:
                    split = 40
                    go = True
                elif mag > 0.5:
                    split = 30
                    go = True
                
                # Adds interpolated data to new list
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
            
            # Reassigns datax and datay
            datax = Xprocess
            datay = Yprocess
            
            # Guess values for Newton Raphson
            theta = [0.1,0.1]
            guess = [math.atan(datay[0]/datax[0]),math.sqrt(datax[0]**2 + datay[0]**2)]
            
            for t in range(len(datax)):
                # Calculates angular position output using Newton Raphson function for each
                # theoretical data point.
                X = [Xprocess[t],datay[t]]
                temp = NewtonRaphson(lambda theta: g(X, theta), dg_dtheta, guess, .01)
                temp[0] = abs(temp[0]%(math.pi))
                temp[1] = abs(temp[1])
                datax[t] = round((temp[0])*NEMA.get() / (2*math.pi))
                datay[t] = round((temp[1])*OTHERSTEP.get() * 100/ (2*math.pi))
                
                guess=temp
            
            state.put(3)
            
        else:
            yield None

def UI():
    global selected
    
    '''
    This task directly interfaces with the user, displaying possible
    HPGL files that they can select by moving and clicking the joystick.
    Any HPGL files loaded on the Nucleo will automatically update on
    the LCD.
    '''
    while True:
        # Startup display of LCD
        if state.get() == 0:
            i = 0
            line1 = filenames[i]
            line2 = filenames[i+1]
            state.put(1)
        
        # Contains all logic to cycle through files on LCD and display
        elif state.get() == 1:
            if (stick.readX() > 3048) and i < len(filenames)-1 and i >= 0:
                i+=1
            elif (stick.readX() < 1048) and i <= len(filenames)-1 and i > 0:
                i-=1
        
            if i == len(filenames)-1 and (i % 2 == 0):
                line1 = filenames[i]
                if (i+1) < len(filenames):
                    line2 = filenames[i+1]
                else:
                    line2 = ""
            elif i <= len(filenames)-1 and not (i % 2 == 1):
                if (i+1) < len(filenames):
                    line1 = filenames[i]
                    line2 = filenames[i+1]
            elif (i % 2 == 1):
                line1 = filenames[i-1]
                line2 = filenames[i]
            
            # Displays selector array on selected file
            if (i % 2 == 0):
                display.lcd_string(">" + line1, display.LCD_LINE_1)
                display.lcd_string(line2, display.LCD_LINE_2)
            else:
                display.lcd_string(line1, display.LCD_LINE_1)
                display.lcd_string(">" + line2, display.LCD_LINE_2)
        
            # Selects file if joystick is pressed
            if (stick.readSwitch()):
                light.on()
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

# This code creates six shares and four tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops.
if __name__ == "__main__":
    print ('Welcome to Pen Plotter, use joystick to select mode/HPGL file')
    '''
    ---------------------Defining Image Processing Functions-----------------------
    '''
    def interp(xpoint,ypoint,num):
        # Linearly interpolates provided data
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
    
    '''
    Creating Shares, Queues, and Tasks
    '''
    # Share to define present state of program
    state = task_share.Share ('h', thread_protect = False, name = "State")
    state.put(0)
    
    # Later used in indexing operations when sending motor data
    ii = task_share.Share ('h', thread_protect = False, name = "Index ii")
    ii.put(0)
    
    # Used on sending motor data
    mode = task_share.Share ('h', thread_protect = False, name = "Mode")
    mode.put(0)
    
    # Number of steps for each rotation of NEMA motor
    NEMA = task_share.Share ('h', thread_protect = False, name = "NEMA Rotation")
    NEMA.put(1600)
    
    # Number of steps for each rotation of old HP motor
    OTHERSTEP = task_share.Share ('h', thread_protect = False, name = "OTHER STEPPER Rotation")
    OTHERSTEP.put(385)
    
    # Contains file name during UI operations
    filetype = task_share.Share ('h', thread_protect = False, name = "File Type")
    
    # Defines tasks to be run at various periods (miliseconds) and priorities
    # with the lowest being the highest priority
    File_Draw = cotask.Task (File_Draw, name = 'File_Draw', priority = 1, 
                         period = 100, profile = True, trace = False)
    Free_Draw = cotask.Task (Free_Draw, name = 'Free_Draw', priority = 2, 
                         period = 100, profile = True, trace = False)
    Calc_HPGL = cotask.Task (Calc_HPGL, name = 'Calc_HPGL', priority = 3, 
                         period = 1000, profile = True, trace = False)
    UI = cotask.Task (UI, name = 'UI', priority = 4, 
                         period = 100, profile = True, trace = False)
 
    # Adds tasks to task exectution list    
    cotask.task_list.append(UI)
    cotask.task_list.append(Calc_HPGL)
    cotask.task_list.append(File_Draw)
    cotask.task_list.append(Free_Draw)
    
    
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()
    
    # Initialize objects from each class.
    light = led.LED()
    pen = solenoid.Solenoid()
    stick = joystick.Joystick()
    display = lcd.LCD()
    
    # Store any HPGL files currently in Nucleo
    selected = ""
    filenames = ["Free Draw"]
    
    for x in os.listdir():
        if x.endswith(".hpgl") and not x.startswith('._'):
            # Prints only text file present in My Folder
            filenames.append(x)
    
        
    # Starts pen in up configuration
    pen.up()
    
    # Defines cancellation button on Nulceo
    CancelButton = Pin(Pin.cpu.C13, mode=Pin.IN, pull=pyb.Pin.PULL_UP)
    
    # Run the scheduler with the chosen scheduling algorithm. Quit if any 
    # character is received through the serial port
    vcp = pyb.USB_VCP ()
    while not vcp.any ():
        cotask.task_list.pri_sched ()
    
    # Empty the comm port buffer of the character(s) just pressed
    vcp.read ()
    