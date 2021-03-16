"""Takes the data sent via LoRa to the Adafruit M0 Featherboard, parses, and displays in a tkinter based GUI

This script will handle the intake and parsing of the hex (char) values to
then display them nicely. If you are going to make your own GUI instead of
using the existing, use the other python script 'read_MSP_serial.py' to start.
This will display the data in a tkinter GUI.
"""

from tkinter import*
import numpy as np
import csv
import serial

__author__ = "Cameron Blegen & Adam Wulfing"
__copyright__ = "Copyright, Montana Space Grant Consortium"
__version__ = "1.1.0"

# GUI
csvFile = 'guiData.csv'

# Parse
start_flag = False
payload = np.zeros(10)  # 0 = internal temp | 1 = external temp | 2 = accel x    | 3 = accel y    | 4 = accel z |
                        # 5 = pressure      | 6 = time          | 7 = GPS coords | 8 = GPS coords | 9 = height in m
np.set_printoptions(suppress=True)
ser = serial.Serial('COM4')  # go to device manager and enter in the COM port desired
print(ser.name)  # Print our port so we know we got the right one

# ***** GUI SETUP START *****
# setup window
window = Tk()

# set variable names
ITvL = Label(window)
ETvL = Label(window)
aXvL = Label(window)
aYvL = Label(window)
aZvL = Label(window)
prvL = Label(window)
tivL = Label(window)
lavL = Label(window)
lovL = Label(window)
atvL = Label(window)


def close_window():
    window.destroy()
    exit()


def windowSetup():   
    
    window. geometry('350x400')
    window.title("LoRa GUI")

    be = Button(window, text="Exit", width=21, bg="red", command=close_window)
    be.grid(row=12, column=0)

    labelIT = Label(window, text = "Internal Temp  :")
    labelET = Label(window, text = "External Temp  :")
    labelaX = Label(window, text = "Acceleration X :")
    labelaY = Label(window, text = "Acceleration Y :")
    labelaZ = Label(window, text = "Acceleration Z :")
    labelpr = Label(window, text = "Pressure            :")
    labelti = Label(window, text = "Time                  :")
    labella = Label(window, text = "Latitude            :")
    labello = Label(window, text = "Longitude         :")
    labelat = Label(window, text = "Altitude              :")
    labelb = Label(window, text = "                ")
    labelbl = Label(window, text = "                ")

    unitIT = Label(window, text = "℃")
    unitET = Label(window, text = "℃")
    unitaX = Label(window, text = "milli gs")
    unitaY = Label(window, text = "milli gs")
    unitaZ = Label(window, text = "milli gs")
    unitpr = Label(window, text = "mBar")
    unitti = Label(window, text = "")
    unitla = Label(window, text = "Degrees")
    unitlo = Label(window, text = "Degrees")
    unitat = Label(window, text = "Meters")

    unitIT.grid(row=0, column = 2)
    unitET.grid(row=1, column = 2)
    unitaX.grid(row=2, column = 2)
    unitaY.grid(row=3, column = 2)
    unitaZ.grid(row=4, column = 2)
    unitpr.grid(row=5, column = 2)
    unitti.grid(row=6, column = 2)
    unitla.grid(row=7, column = 2)
    unitlo.grid(row=8, column = 2)
    unitat.grid(row=9, column = 2)
    
    labelIT.grid(row=0, column = 0)
    labelET.grid(row=1, column = 0)
    labelaX.grid(row=2, column = 0)
    labelaY.grid(row=3, column = 0)
    labelaZ.grid(row=4, column = 0)
    labelpr.grid(row=5, column = 0)
    labelti.grid(row=6, column = 0)
    labella.grid(row=7, column = 0)
    labello.grid(row=8, column = 0)
    labelat.grid(row=9, column = 0)
    labelb.grid(row=10, column = 0)
    labelbl.grid(row=11, column = 0)
    #
    ITvL.grid(row=0,column = 1)
    ETvL.grid(row=1,column = 1)
    aXvL.grid(row=2,column = 1)
    aYvL.grid(row=3,column = 1)
    aZvL.grid(row=4,column = 1)
    prvL.grid(row=5,column = 1)
    tivL.grid(row=6,column = 1)
    lavL.grid(row=7,column = 1)
    lovL.grid(row=8,column = 1)
    atvL.grid(row=9,column = 1)

    headers = ['iTemp', 'eTemp', 'X', 'Y', 'Z', 'pressure','time','GPS coord','GPS coord', 'Alt']  
    print("opening csv")
    with open(csvFile, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        f.close()
    print("CSV closed")
# ***** GUI SETUP STOP *****


def loop():
    print("start loop")
    global start_flag

    # ***** START OF PARSE SCRIPT *****
    while 1:
        try:
            start_flag = ser.read().decode("utf-8") == '['  # this is our start condition, this MUST be sent from LoRa for this to work
        except:
            pass
            # print("couldn't decode character (this is okay)")  # This line can be used for debug if desired
        if start_flag:
            print("reading byte")
            ex = ser.read(14)  # read the numerical values from the serial port
            a = list(ex)  # turn bytes into a list

            # ------- GET SENSOR VALUES AND PUT IN PAYLOAD -------
            for i in range(5):  # loop through the first 5 values and assign them to the "payload"
                # get the two byte value and assign
                payload[i] = a[i+i] << 8
                payload[i] += a[i+i+1]
        
                # this if statement handles if the number is negative (2s complement)
                if payload[i].astype(int) >> 15:
                    temp_list = list(bin(payload[i].astype(int)))  # this is needed to do binary 2s complement
                    for j in range(len(temp_list) - 2):
                        # Flip bits
                        if temp_list[j+2] == '1':
                            temp_list[j+2] = '0'
                        else:
                            temp_list[j+2] = '1'
                    temp_list = "".join(temp_list)  # join the list back together
                    payload[i] = ~ int(temp_list, 2)  # convert the temp list back to a number
                if i < 2:
                    payload[i] = payload[i]/(2**7)  # convert temperature
                if 1 < i < 5:
                    payload[i] = payload[i]*0.061  # convert accelerometer
            payload[5] = (a[10] << 24) + (a[11] << 16) + (a[12] << 8) + a[13]
            payload[5] = payload[5]/100

            # ------- GET GPS VALUES AND PUT IN PAYLOAD -------
            ex = ser.read(117)
            split_gps = str(ex).split(',')

            payload[6] = split_gps[1]  # Get the time

            # Get the N/S coordinate and convert to Decimal Degree form
            payload[7] = float(split_gps[2])/100
            temp_dec = payload[7] % 1
            payload[7] -= temp_dec
            temp_dec = (temp_dec*100)/60
            payload[7] += temp_dec
            if split_gps[3] == 'S':  # Check what N/S hemisphere we are in
                payload[7] = payload[7] * -1  # Set the right negative value if in S hemisphere

            # Get the E/W coordinate and convert to Decimal Degree form
            payload[8] = float(split_gps[4])/100
            temp_dec = payload[8] % 1
            payload[8] -= temp_dec
            temp_dec = (temp_dec*100)/60
            payload[8] += temp_dec
            if split_gps[5] == 'W':  # Check what W/E hemisphere we are in
                payload[8] = payload[8] * -1  # Set the right negative value if in W hemisphere

            payload[9] = split_gps[9]  # Get height of board in m
            
            print(payload)
            print(split_gps)
            # ***** STOP OF PARSE SCRIPT *****

            # ***** START OF GUI SCRIPT *****
            # hour = int(str(timehms[:2]))
            # minute = timehms[2:4]
            # second = timehms[4:6]
            
            ITvL.config(text = payload[0])
            ETvL.config(text = payload[1])
            aXvL.config(text = payload[2])
            aYvL.config(text = payload[3])
            aZvL.config(text = payload[4])
            prvL.config(text = payload[5])
            tivL.config(text = payload[6])
            # text = hour + ":" + minute + ":" + second)
            lavL.config(text = payload[7])
            lovL.config(text = payload[8])
            atvL.config(text = payload[9])

            # read to csv and save
            with open(csvFile, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(payload)
                f.close()
            # update gui with new values
            print("values updated")
            window.update()
            # window.after(10)
            # ***** STOP OF GUI SCRIPT *****


windowSetup()
loop()
window.mainloop()


