#!/usr/bin/python

# Copyright 2012 Daniel Berlin (with some changes by Adafruit Industries/Limor Fried)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal  MCP230XX_GPIO(1, 0xin
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
# of the Software, and to permit persons to whom the Software is furnished to do
# so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from Adafruit_I2C import Adafruit_I2C
import smbus
import time
import spidev
import time
import os
import threading
from threading import Thread
import RPi.GPIO as GPIO
import time

MCP23017_IODIRA = 0x00
MCP23017_IODIRB = 0x01
MCP23017_GPIOA  = 0x12
MCP23017_GPIOB  = 0x13
MCP23017_GPPUA  = 0x0C
MCP23017_GPPUB  = 0x0D
MCP23017_OLATA  = 0x14
MCP23017_OLATB  = 0x15
MCP23008_GPIOA  = 0x09
MCP23008_GPPUA  = 0x06
MCP23008_OLATA  = 0x0A

class Adafruit_MCP230XX(object):
    OUTPUT = 0
    INPUT = 1

    def __init__(self, address, num_gpios, busnum=-1):
        assert num_gpios >= 0 and num_gpios <= 16, "Number of GPIOs must be between 0 and 16"
        self.i2c = Adafruit_I2C(address=address, busnum=busnum)
        self.address = address
        self.num_gpios = num_gpios

        # set defaults
        if num_gpios <= 8:
            self.i2c.write8(MCP23017_IODIRA, 0xFF)  # all inputs on port A
            self.direction = self.i2c.readU8(MCP23017_IODIRA)
            self.i2c.write8(MCP23008_GPPUA, 0x00)
        elif num_gpios > 8 and num_gpios <= 16:
            self.i2c.write8(MCP23017_IODIRA, 0xFF)  # all inputs on port A
            self.i2c.write8(MCP23017_IODIRB, 0xFF)  # all inputs on port B
            self.direction = self.i2c.readU8(MCP23017_IODIRA)
            self.direction |= self.i2c.readU8(MCP23017_IODIRB) << 8
            self.i2c.write8(MCP23017_GPPUA, 0x00)
            self.i2c.write8(MCP23017_GPPUB, 0x00)

    def _changebit(self, bitmap, bit, value):
        assert value == 1 or value == 0, "Value is %s must be 1 or 0" % value
        if value == 0:
            return bitmap & ~(1 << bit)
        elif value == 1:
            return bitmap | (1 << bit)

    def _readandchangepin(self, port, pin, value, currvalue = None):
        assert pin >= 0 and pin < self.num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.num_gpios)
        #assert self.direction & (1 << pin) == 0, "Pin %s not set to output" % pin
        if not currvalue:
             currvalue = self.i2c.readU8(port)
        newvalue = self._changebit(currvalue, pin, value)
        self.i2c.write8(port, newvalue)
        return newvalue


    def pullup(self, pin, value):
        if self.num_gpios <= 8:
            return self._readandchangepin(MCP23008_GPPUA, pin, value)
        if self.num_gpios <= 16:
            lvalue = self._readandchangepin(MCP23017_GPPUA, pin, value)
            if (pin < 8):
                return
            else:
                return self._readandchangepin(MCP23017_GPPUB, pin-8, value) << 8

    # Set pin to either input or output mode
    def config(self, pin, mode):
        if self.num_gpios <= 8:
            self.direction = self._readandchangepin(MCP23017_IODIRA, pin, mode)
        if self.num_gpios <= 16:
            if (pin < 8):
                self.direction = self._readandchangepin(MCP23017_IODIRA, pin, mode)
            else:
                self.direction |= self._readandchangepin(MCP23017_IODIRB, pin-8, mode) << 8

        return self.direction

    def output(self, pin, value):
        # assert self.direction & (1 << pin) == 0, "Pin %s not set to output" % pin
        if self.num_gpios <= 8:
            self.outputvalue = self._readandchangepin(MCP23008_GPIOA, pin, value, self.i2c.readU8(MCP23008_OLATA))
        if self.num_gpios <= 16:
            if (pin < 8):
                self.outputvalue = self._readandchangepin(MCP23017_GPIOA, pin, value, self.i2c.readU8(MCP23017_OLATA))
            else:
                self.outputvalue = self._readandchangepin(MCP23017_GPIOB, pin-8, value, self.i2c.readU8(MCP23017_OLATB)) << 8

        return self.outputvalue


        self.outputvalue = self._readandchangepin(MCP23017_IODIRA, pin, value, self.outputvalue)
        return self.outputvalue

    def input(self, pin):
        assert pin >= 0 and pin < self.num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.num_gpios)
        assert self.direction & (1 << pin) != 0, "Pin %s not set to input" % pin
        if self.num_gpios <= 8:
            value = self.i2c.readU8(MCP23008_GPIOA)
        elif self.num_gpios > 8 and self.num_gpios <= 16:
            value = self.i2c.readU8(MCP23017_GPIOA)
            value |= self.i2c.readU8(MCP23017_GPIOB) << 8
        return value & (1 << pin)

    def readU8(self):
        result = self.i2c.readU8(MCP23008_OLATA)
        return(result)

    def readS8(self):
        result = self.i2c.readU8(MCP23008_OLATA)
        if (result > 127): result -= 256
        return result

    def readU16(self):
        assert self.num_gpios >= 16, "16bits required"
        lo = self.i2c.readU8(MCP23017_OLATA)
        hi = self.i2c.readU8(MCP23017_OLATB)
        return((hi << 8) | lo)

    def readS16(self):
        assert self.num_gpios >= 16, "16bits required"
        lo = self.i2c.readU8(MCP23017_OLATA)
        hi = self.i2c.readU8(MCP23017_OLATB)
        if (hi > 127): hi -= 256
        return((hi << 8) | lo)

    def write8(self, value):
        self.i2c.write8(MCP23008_OLATA, value)

    def write16(self, value):
        assert self.num_gpios >= 16, "16bits required"
        self.i2c.write8(MCP23017_OLATA, value & 0xFF)
        self.i2c.write8(MCP23017_OLATB, (value >> 8) & 0xFF)

# RPi.GPIO compatible interface for MCP23017 and MCP23008

class MCP230XX_GPIO(object):
    OUT = 0
    IN = 1
    BCM = 0
    BOARD = 0
    def __init__(self, busnum, address, num_gpios):
        self.chip = Adafruit_MCP230XX(address, num_gpios, busnum)
    def setmode(self, mode):
        # do nothing
        pass
    def setup(self, pin, mode):
        self.chip.config(pin, mode)
    def input(self, pin):
        return self.chip.input(pin)
    def output(self, pin, value):
        self.chip.output(pin, value)
    def pullup(self, pin, value):
        self.chip.pullup(pin, value)

#!/usr/bin/python
#
# HD44780 LCD Test Script for
# Raspberry Pi
#
# Author : Matt Hawkins
# Site   : http://www.raspberrypi-spy.co.uk
#
# Date   : 26/07/2012
#
 
# The wiring for the LCD is as follows:
# 1 : GND
# 2 : 5V
# 3 : Contrast (0-5V)*
# 4 : RS (Register Select)
# 5 : R/W (Read Write)       - GROUND THIS PIN
# 6 : Enable or Strobe
# 7 : Data Bit 0             - NOT USED
# 8 : Data Bit 1             - NOT USED
# 9 : Data Bit 2             - NOT USED
# 10: Data Bit 3             - NOT USED
# 11: Data Bit 4
# 12: Data Bit 5
# 13: Data Bit 6
# 14: Data Bit 7
# 15: LCD Backlight +5V**
# 16: LCD Backlight GND
 
GPIO.setwarnings(False)
# Define GPIO to LCD mapping
LCD_RS = 17
LCD_E  = 18
LCD_D4 = 22
LCD_D5 = 23
LCD_D6 = 24
LCD_D7 = 25
 
# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
 
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line 
 
# Timing constants
E_PULSE = 0.0001
E_DELAY = 0.00005


def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD)
  lcd_byte(0x32,LCD_CMD)
  lcd_byte(0x28,LCD_CMD)
  lcd_byte(0x0C,LCD_CMD)
  lcd_byte(0x06,LCD_CMD)
  lcd_byte(0x01,LCD_CMD)  
 
def lcd_string(message):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")  
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
 
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
 
  GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)      
 
  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY) 




if __name__ == '__main__':
    def analogue_input(analogue0,analogue1,analogue2,analogue3,analogue4,analogue5,analogue6,analogue7, ):
        global analogue_out0
        global analogue_out1
        global analogue_out2
        global analogue_out3
        global analogue_out4
        global analogue_out5
        global analogue_out6
        global analogue_out7
        while(True):
            analogue_out0 = ReadChannel(analogue0)
            analogue_out1 = ReadChannel(analogue1)
            analogue_out2 = ReadChannel(analogue2)
            analogue_out3 = ReadChannel(analogue3)
            analogue_out4 = ReadChannel(analogue4)
            analogue_out5 = ReadChannel(analogue5)
            analogue_out6 = ReadChannel(analogue6)
            analogue_out7 = ReadChannel(analogue7)
            time.sleep(0.5);
    
    # Function to read SPI data from MCP3008 chip
    # Channel must be an integer 0-7
    def ReadChannel(channel):
        adc = spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
 
    # Define sensor channels
    analogue0  = 0
    analogue1  = 1
    analogue2  = 2
    analogue3  = 3
    analogue4  = 4
    analogue5  = 5
    analogue6  = 6
    analogue7  = 7
    # Open SPI bus
    spi = spidev.SpiDev()
    spi.open(0,0)
    
    def welcome():
        lcd_byte(LCD_LINE_1, LCD_CMD)
        lcd_string("  Pi & Bash >_ ")
        lcd_byte(LCD_LINE_2, LCD_CMD)
        lcd_string("RPi Add-On Board")
        time.sleep(0.5)
        lcd_byte(LCD_LINE_1, LCD_CMD)
        lcd_string("  Pi & Bash > ")
        lcd_byte(LCD_LINE_2, LCD_CMD)
        lcd_string("RPi Add-On Board")
        time.sleep(0.5)
    
    GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
    GPIO.setup(LCD_E, GPIO.OUT)  # E
    GPIO.setup(LCD_RS, GPIO.OUT) # RS
    GPIO.setup(LCD_D4, GPIO.OUT) # DB4
    GPIO.setup(LCD_D5, GPIO.OUT) # DB5
    GPIO.setup(LCD_D6, GPIO.OUT) # DB6
    GPIO.setup(LCD_D7, GPIO.OUT) # DB7
 
    # Initialise display
    lcd_init()
    
    # ***************************************************
    # Set num_gpios to 8 for MCP23008 or 16 for MCP23017!
    # ***************************************************
    # mcp = Adafruit_MCP230XX(address = 0x20, num_gpios = 8) # MCP23008
    mcp = Adafruit_MCP230XX(address = 0x20, num_gpios = 16) # MCP23017

    # **************************************************************************
    # Digital I/O Configuration Section
    # **************************************************************************
    
    # This section contains the configuraiton for the 8 pre-installed MCP23017
    # Inputs (5 pushbuttons) and Outputs (3 LEDs).
    
    # Outputs:
    mcp.config(0, mcp.OUTPUT) # LCD Backlight
    mcp.config(8, mcp.OUTPUT) # Green LED
    mcp.config(10, mcp.OUTPUT) # Amber LED
    mcp.config(12, mcp.OUTPUT) # Red LED
    
    
    # Inputs:
    mcp.config(15, mcp.INPUT) # LCD Lower Line Select Button
    mcp.pullup(15, 1)
    mcp.config(14, mcp.INPUT) # LCD Upper Line Select Button
    mcp.pullup(14, 1)
    mcp.config(13, mcp.INPUT) # Down Button
    mcp.pullup(13, 1)
    mcp.config(11, mcp.INPUT) # Enter Button
    mcp.pullup(11, 1)
    mcp.config(9, mcp.INPUT) # Up Button
    mcp.pullup(9, 1)
    
    #***************************************************************************
    #***************************************************************************
    #***************************************************************************
    # This section is where you can modify the configuraiton for the 8 remaining
    # MCP23017 Input/Outputs.
    
    # OUTPUTS:
    # Remove the # character from any I/O lines which
    # you want to configure as an output
    #mcp.config(1, mcp.OUTPUT)
    #mcp.config(2, mcp.OUTPUT)
    #mcp.config(3, mcp.OUTPUT)
    #mcp.config(4, mcp.OUTPUT)
    #mcp.config(5, mcp.OUTPUT)
    #mcp.config(6, mcp.OUTPUT)
    #mcp.config(7, mcp.OUTPUT)   
    
    # INPUTS: 
    # Remove the # character from both lines associated with any I/O lines which
    # you want to configure as an input
    #mcp.config(0, mcp.INPUT)
    #mcp.pullup(0, 1)
    #mcp.config(1, mcp.INPUT)
    #mcp.pullup(1, 1)
    #mcp.config(2, mcp.INPUT)
    #mcp.pullup(2, 1)
    #mcp.config(3, mcp.INPUT)
    #mcp.pullup(3, 1)
    #mcp.config(4, mcp.INPUT)
    #mcp.pullup(4, 1)
    #mcp.config(5, mcp.INPUT)
    #mcp.pullup(5, 1)
    #mcp.config(6, mcp.INPUT)
    #mcp.pullup(6, 1)
    #mcp.config(7, mcp.INPUT)
    #mcp.pullup(7, 1)
    #***************************************************************************
    
    
    
    
    
    
    
    #***************************************************************************
    # The Programming Starts Here:
    print "Press Control+z to quit at any time."
    
    # Start Analogue Data Capture Thread
    analogue_thread = Thread(target = analogue_input, args = (analogue0,analogue1,analogue2,analogue3,analogue4,analogue5,analogue6,analogue7, ))
    analogue_thread.start() 

    
    # Start by displaying "Pi & Bash" and "RPi Add-On Board" on the LCD
    mcp.output(8, 0)
    mcp.output(10, 0)
    mcp.output(12, 0)
    mcp.output(0, 1)
    lcd_init()
    welcome()
    welcome()
    welcome()
    welcome()
    welcome()
    lcd_init()
    # Display "www.pinventor.co.uk" on the LCD
    lcd_byte(LCD_LINE_1, LCD_CMD)
    lcd_string("www.pinventor")
    lcd_byte(LCD_LINE_2, LCD_CMD)
    lcd_string(".co.uk")
    # Wait for 3 seconds
    time.sleep(3)
    lcd_init()
    

    while (True): # The code below indented from the while (True) command is looped forever
        
        while (((mcp.input(15) >> 15)+(mcp.input(14) >> 14)+(mcp.input(13) >> 13)+(mcp.input(11) >> 11)+(mcp.input(9) >> 9))<4): # Detect if more than one button is pressed
            temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
            lcd_byte(LCD_LINE_1, LCD_CMD)
            lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
            lcd_byte(LCD_LINE_2, LCD_CMD)
            lcd_string("Multiple Buttons")
            
        while (((mcp.input(15) >> 15)+(mcp.input(14) >> 14)+(mcp.input(13) >> 13)+(mcp.input(11) >> 11)+(mcp.input(9) >> 9)) == 5): # Detect if no buttons are pressed
            temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
            lcd_byte(LCD_LINE_1, LCD_CMD)
            lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
            lcd_byte(LCD_LINE_2, LCD_CMD)
            lcd_string("Press Something")
           
        while (((mcp.input(15) >> 15)+(mcp.input(14) >> 14)+(mcp.input(13) >> 13)+(mcp.input(11) >> 11)+(mcp.input(9) >> 9)) == 4): # Detect if one button is pressed
            if (mcp.input(15) >> 15) == 0:
                lcd_init()
                temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("LCD LOWER BUTTON")
                time.sleep(1);
            if (mcp.input(14) >> 14) == 0:
                lcd_init()
                temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("LCD UPPER BUTTON")
                time.sleep(1);
            if (mcp.input(13) >> 13) == 0:
                lcd_init()
                temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("DOWN BUTTON")
                time.sleep(1);
            if (mcp.input(9) >> 9) == 0:
                lcd_init()
                temperature = ((analogue_out0 * 472)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("UP BUTTON")
                time.sleep(1);
            if (mcp.input(11) >> 11) == 0:
                lcd_init()
                temperature = ((analogue_out0 * 465)/float(1023))-50 # this line converts the anlaogue0 input from the TMP36 sensor into a temperature in deg C.
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temperature=" + (str(int(temperature))) + "C") # Display the temperature from the TMP36 sensor on the first line of the display.
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("ENTER BUTTON")
                
                # Make the LEDs Flash for a while when the Enter Button is Pressed
                mcp.output(0, 0)
                mcp.output(8, 1)
                time.sleep(0.2);
                mcp.output(10, 1)
                time.sleep(0.2);
                mcp.output(12, 1)
                time.sleep(0.2);                
                mcp.output(0, 1)
                mcp.output(8, 0)
                mcp.output(10, 0)
                mcp.output(12, 0)
                time.sleep(0.2);
                mcp.output(8, 1)
                time.sleep(0.2);
                mcp.output(10, 1)
                time.sleep(0.2);
                mcp.output(0, 0)
                mcp.output(12, 1)
                time.sleep(0.2);                
                mcp.output(8, 0)
                mcp.output(10, 0)
                mcp.output(12, 0)
                time.sleep(0.2);
                mcp.output(8, 1)
                time.sleep(0.2);
                mcp.output(10, 1)
                time.sleep(0.2);
                mcp.output(0, 1)
                mcp.output(12, 1)
                time.sleep(0.2);                
                mcp.output(8, 0)
                mcp.output(10, 0)
                mcp.output(12, 0)
                time.sleep(0.2);
                mcp.output(8, 1)
                time.sleep(0.2);
                mcp.output(0, 0)
                mcp.output(10, 1)
                time.sleep(0.2);
                mcp.output(12, 1)
                time.sleep(0.2);                
                mcp.output(8, 0)
                mcp.output(10, 0)
                mcp.output(12, 0)
                mcp.output(0, 1)
