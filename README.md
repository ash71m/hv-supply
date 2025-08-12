# High Voltage Supply Board Code 
This repository includes all the code needed to program and test the high voltage supply board that provides up to 80 V in order to operate SiPMs. The board can be controlled with the on-board RP2040 chip or with an external microcontroller through UART communication.

When using the V01 board, use main.py.  You will need to install this driver on to the board: https://github.com/robert-hh/ads1x15
When using the V0.3  board, use main_v3.py and install the provided drivers onto the board. main_v3.py is still in progress. Voltage setting is functional; voltage reading is not fully functional.


## Features
  - Input: 5 V at 200 mA
  - Output: 85 V at 10 mA
  - Voltage Setting Accuracy: 0.01 V
  - I2C connection between internal components
  - Supports UART communication with external microcontroller
    
## Instructions

### Native Control
1. **Flash the firmware**
    - Flash MicroPython on to the RP2040 by downloading the correct firmware.
    - Hold down the BOOT button on the board as you plug it in to your computer with the USB cable.
    - Drag the .uf2 file into the RPI-RP2 folder that appears.
    - Release the boot button and confirm that the RP2040 no longer appears as a removable disk on your computer.

2. **Install Thonny IDE (or other)**
    - Thonny is the preferred IDE for this project but any MicroPython IDE will work

3. **Place main.py into the RP2040-Zero**
    - Using the Thonny interface, open and save the main.py file onto the board. When you click the run button, you will be able to type commands into the command line. See __ for list of commands.
    - Make note of the name of the serial port associated with the board in THonny under Tools > Options > Interpreter (i.e. COM3)

4. **Run the GUI**
    - Open the interface.py code in your preferred IDE. Change the ser.port variable to the correct serial port.
    - Run the interface and use the buttons to send commands. You will be able to see a log of commands sent and received printed in the terminal of your IDE.

### External Control
1. **Set up the RP2040**
     - Follow steps 1-3 for the RP2040.
2. **Connect the boards**
     - Connect the RX, TX, and GND pins of your external board to the labelled pins on the HV supply board.
3. **Change the control in the software**
     - Set the *external* variable at the top of main.py (in RP2040-Zero) to True.
4. **Set up the external board**
     -  Place main_host.py in your external board. Change the file name to main.py to ensure it runs on startup.
     -  Change the pin numbers for TX and RX to the correct ones connected on your board.
     -  Run the code and test sending and receiving commands from your external board. 
4. **Set up the GUI**
     - In interface.py, change the ser.port variable to the serial port name of your external board and run the GUI as before. 
