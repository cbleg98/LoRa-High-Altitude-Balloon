# LoRa-High-Altitude-Balloon
This is the respository for Montana State University LoRa group capstone project for the Montana Space Grant Consortium BOREALIS lab. The following code runs on a custom board designed to gather atmospheric data on-board a high-altitude balloon payload. This is an MSP430FR2355 based project which uses SPI, UART and I2C. The data is then transmitted over long range (LoRa) radio. For board info contact Montana Space Grant Consortium at Montana State University.

## Structure
LoRa-High-Altitude-Balloon/project_folders/main.c <-- Each project contains the firmware (and included header.h files) needed to run different parts of the board.
LoRa-High-Altitude-Balloon/GUI_scripts <-- This folder contains the scripts for the GUI/parse code (python)

### Firmware
Each of the folders contain a different piece of the firmware puzzle. All of these pieces are finalized in the "Firmware V1" folder which will work turn-key when flashed to the custom MSGC board.

### Software
There is also included a "parse" script as well as a GUI display script. This helps visualize the data on a serial port and/or tkinter Python GUI.

## Programming
Code Composer Studio was used for the development and programming of this firmware

# Questions?
Contact Cameron Blegen or Montana Space Grant Consortium at Montana State

## Contributors
- **Cameron Blegen**
- **Larson Brandstetter**
- **Adam Wulfing**

## Special Thanks
- **Randal Larimer**
- **Montana Space Grant Consortium**
- **Montana State University BOREALIS lab**
- **Montana State University**

