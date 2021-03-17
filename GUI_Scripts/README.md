# GUI CODE
This code will display the results of our board sending data and the ground station receiving it.

## GUI 3.1
GUI 3.1 is the most up-to-date and working version. Use this to receive data and display it in a nice to read way.

**NOTE:** You can use the global variable in this script "DEBUG_ON = True/False" to see more verbose data, turn to False for
a faster and lighterweight program.

## read_MSP_serial.py
This can be used to simply receive the data sent by the Arduino code and view it, does not create/send to a GUI. This is co-maintained
with the GUI 3.1 but may be missing a few smaller things, mostly helpful for debugging/starting point/knowing if your code is running
properly and receiving data.