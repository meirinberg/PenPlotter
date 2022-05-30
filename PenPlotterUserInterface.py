import pyb
from pyb import Pin,Timer,ADC,ExtInt
import time
import os

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# Setup Pins
XPIN = Pin(Pin.cpu.A5, mode=Pin.IN)
YPIN = Pin(Pin.cpu.A6, mode=Pin.IN)
BUTTONPIN = Pin(Pin.cpu.C5, mode=Pin.IN, pull=pyb.Pin.PULL_UP)
LEDPIN = Pin(Pin.cpu.C4, mode=Pin.OUT)

LCD_E = Pin(Pin.cpu.B9, mode=Pin.OUT)
LCD_RS = Pin(Pin.cpu.C9, mode=Pin.OUT)
RWPIN = Pin(Pin.cpu.B8, mode=Pin.OUT)
LCD_D4 = Pin(Pin.cpu.C8, mode=Pin.OUT)
LCD_D5 = Pin(Pin.cpu.A10, mode=Pin.OUT)
LCD_D6 = Pin(Pin.cpu.B3, mode=Pin.OUT)
LCD_D7 = Pin(Pin.cpu.B5, mode=Pin.OUT)

def lcd_init():
  # Initialise display
#   lcd_byte(0x30,LCD_CMD) # 110011 Initialise
#   time.sleep(E_DELAY*10)
#   lcd_byte(0x30,LCD_CMD) # 110011 Initialise
#   time.sleep(E_DELAY)
#   lcd_byte(0x30,LCD_CMD) # 110011 Initialise
#   time.sleep(E_DELAY)
#   lcd_byte(0x02,LCD_CMD) # 110010 Initialise
#   lcd_byte(0x28,LCD_CMD) # 110010 Initialise
#   lcd_byte(0x10,LCD_CMD) # 110010 Initialise
#   lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
#   lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
#   lcd_byte(0x14,LCD_CMD) # 101000 Data length, number of lines, font size
#   lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  
  #LCD_D6.high()
  
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
  
  

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
  if mode:
      LCD_RS.high() # RS
  else:
      LCD_RS.low() # RS
  #print("{0:b}".format(bits))
  upper = ((bits>>4)&0x0f);
  #print("{0:b}".format(upper), end = " ")
  # High bits
  LCD_D4.low()
  LCD_D5.low()
  LCD_D6.low()
  LCD_D7.low()
  if upper&0b00000001==0b00000001:
      LCD_D4.high()
  if upper&0b00000010==0b00000010:
      LCD_D5.high()
  if upper&0b00000100==0b00000100:
      LCD_D6.high()
  if upper&0b00001000==0b00001000:
      LCD_D7.high()
  
  # Toggle 'Enable' pin
  lcd_toggle_enable()
  lower = ((bits)&0x0f);
  #print("{0:b}".format(lower))
  #print(LCD_D4.value(), LCD_D5.value(), LCD_D6.value(), LCD_D7.value(), end = " ")
  # Low bits
  LCD_D4.low()
  LCD_D5.low()
  LCD_D6.low()
  LCD_D7.low()
  if lower&0b00000001==0b00000001:
      LCD_D4.high()
  if lower&0b00000010==0b00000010:
      LCD_D5.high()
  if lower&0b00000100==0b00000100:
      LCD_D6.high()
  if lower&0b00001000==0b00001000:
      LCD_D7.high()
  #print(LCD_D4.value(), LCD_D5.value(), LCD_D6.value(), LCD_D7.value())
  
  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  LCD_E.high()
  time.sleep(E_PULSE)
  LCD_E.low()
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display
  MESSAGE_SPACE = LCD_WIDTH-len(message)
  i=0
  while i<MESSAGE_SPACE:
      message = message + " "
      i+=1 
  lcd_byte(line, LCD_CMD)
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
    filenames = ["Free Draw"]
    for x in os.listdir():
        if x.endswith(".hpgl") and not x.startswith('._'):
            # Prints only text file present in My Folder
            filenames.append(x)

    RWPIN.low()
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    lcd_init()
#     lcd_string("The LCD display",LCD_LINE_1)
#     lcd_string("now works!!",LCD_LINE_2)

    # Assign Pins
    adcX = pyb.ADC(XPIN)                  # create an analog object from a pin
    adcY = pyb.ADC(YPIN)                  # create an analog object from a pin
    
    i = 0
    line1 = filenames[i]
    line2 = filenames[i+1]
    while True:
        if (adcX.read() > 3048) and i < len(filenames)-1 and i >= 0:
            print("DOWN")
            i+=1
        elif (adcX.read() < 1048) and i <= len(filenames)-1 and i > 0:
            print("UP")
            i-=1
        
        if i == len(filenames)-1 and (i % 2 == 0):
            line1 = filenames[i]
            if (i+1) < len(filenames):
                line2 = filenames[i+1]
            else:
                line2 = ""
        elif i <= len(filenames)-1 and not i == 1:
            if (i+1) < len(filenames):
                line1 = filenames[i]
                line2 = filenames[i+1]
        elif i == 1:
            line1 = filenames[i-1]
            line2 = filenames[i]
            
        
        if (i % 2 == 0):
            lcd_string(">" + line1, LCD_LINE_1)
            lcd_string(line2, LCD_LINE_2)
        else:
            lcd_string(line1, LCD_LINE_1)
            lcd_string(">" + line2, LCD_LINE_2)
        #print("switch:", not BUTTONPIN.value())
        
        if (not BUTTONPIN.value()):
            LEDPIN.high()
            print("Selected:", filenames[i])
            #lcd_byte(0b00000001,LCD_CMD) # 000001 Clear display
            #time.sleep(3)
        else:
            LEDPIN.low()
            
        #time.sleep(1)

if __name__ == "__main__":
    main()




  
