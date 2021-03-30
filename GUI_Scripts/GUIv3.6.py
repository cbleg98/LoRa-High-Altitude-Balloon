"""Takes the data sent via LoRa to the Adafruit M0 Featherboard, parses, and displays in a tkinter based GUI

This script will handle the intake and parsing of the hex (char) values to
then display them nicely. If you are going to make your own GUI instead of
using the existing, use the other python script 'read_MSP_serial.py' to start.
This will display the data in a tkinter GUI.
"""

from tkinter import *
import numpy as np
import csv
import serial

__author__ = "Cameron Blegen & Adam Wulfing"
__copyright__ = "Copyright, Montana Space Grant Consortium"
__version__ = "3.1.1"

# DEBUG
DEBUG_ON = True

# GUI
csvFile = 'guiData.csv'

# Parse
start_flag = False
found_gga = False  # used for getting the time (due to parse errors)
i = 0
string_rec = ""
payload = np.zeros(500)
sensor_data = np.zeros(13)  # 0 = internal temp | 1 = external temp | 2 = accel x    | 3 = accel y    | 4 = accel z |
                            # 5 = pressure      | 6 = time          | 7 = GPS coords | 8 = GPS coords | 9 = height in m |
                            # 10 = pressure temp | 11 = Packets sent count | 12 = packets received count
# NOTE, the gps coordinate format is converted from DDM to DD. Coordinates are given in decimal degrees.
sensor_data[12] = 0

np.set_printoptions(suppress=True)
ser = serial.Serial('COM5')  # go to device manager and enter in the COM port desired
if DEBUG_ON:
    print(ser.name)  # Print our port so we know we got the right one

# ***** GUI SETUP START *****
# setup window
window = Tk()

# set variable names
ITvL = Label(window)
ETvL = Label(window)
ptvL = Label(window)
aXvL = Label(window)
aYvL = Label(window)
aZvL = Label(window)
pevL = Label(window)
tivL = Label(window)
lavL = Label(window)
lovL = Label(window)
atvL = Label(window)
rivL = Label(window)
snvL = Label(window)
prvL = Label(window)
psvL = Label(window)

def close_window():
    window.destroy()
    exit()


def windowSetup():
    window.geometry('445x515')
    window.title("LoRa GUI")

    be = Button(window, text="Exit", width=21, bg="red", command=close_window)
    be.grid(row=16, column=1)

    labelIT = Label(window, text="Internal Temp",font=("Helvetica",16))
    labelET = Label(window, text="External Temp",font=("Helvetica",16))
    labelpt = Label(window, text="pressure Temp",font=("Helvetica",16))
    labelaX = Label(window, text="Acceleration X",font=("Helvetica",16))
    labelaY = Label(window, text="Acceleration Y",font=("Helvetica",16))
    labelaZ = Label(window, text="Acceleration Z",font=("Helvetica",16))
    labelpe = Label(window, text="Pressure",font=("Helvetica",16))
    labelti = Label(window, text="Time",font=("Helvetica",16))
    labella = Label(window, text="Latitude",font=("Helvetica",16))
    labello = Label(window, text="Longitude",font=("Helvetica",16))
    labelat = Label(window, text="Altitude",font=("Helvetica",16))
    labelri = Label(window, text="RSSI",font=("Helvetica",16))
    labelsn = Label(window, text="Signal to Noise", font=("Helvetica", 16))
    labelps = Label(window, text="Packets Sent", font=("Helvetica", 16))
    labelpr = Label(window, text="Packets Recieved", font=("Helvetica", 16))
    labelb = Label(window, text="                ",font=("Helvetica",16))
    #labelbl = Label(window, text="                ",font=("Helvetica",16))
   # labelex = Label(window, text="                ",font=("Helvetica",16))#space for exit


    unitIT = Label(window, text="℃",font=("Helvetica",12))
    unitET = Label(window, text="℃",font=("Helvetica",12))
    unitpt = Label(window, text="℃",font=("Helvetica",12))
    unitaX = Label(window, text="milli gs",font=("Helvetica",12))
    unitaY = Label(window, text="milli gs",font=("Helvetica",12))
    unitaZ = Label(window, text="milli gs",font=("Helvetica",12))
    unitpe = Label(window, text="mBar",font=("Helvetica",12))
    unitti = Label(window, text="UTC",font=("Helvetica",12))
    unitla = Label(window, text="Degrees",font=("Helvetica",12))
    unitlo = Label(window, text="Degrees",font=("Helvetica",12))
    unitat = Label(window, text="Meters",font=("Helvetica",12))
    unitri = Label(window, text="dB",font=("Helvetica",12))

    unitIT.grid(row=0, column=2)
    unitET.grid(row=1, column=2)
    unitpt.grid(row=2, column=2)
    unitaX.grid(row=3, column=2)
    unitaY.grid(row=4, column=2)
    unitaZ.grid(row=5, column=2)
    unitpe.grid(row=6, column=2)
    unitti.grid(row=7, column=2)
    unitla.grid(row=8, column=2)
    unitlo.grid(row=9, column=2)
    unitat.grid(row=10, column=2)
    unitri.grid(row=11, column=2)

    labelIT.grid(row=0, column=0)
    labelET.grid(row=1, column=0)
    labelpt.grid(row=2, column=0)
    labelaX.grid(row=3, column=0)
    labelaY.grid(row=4, column=0)
    labelaZ.grid(row=5, column=0)
    labelpe.grid(row=6, column=0)
    labelti.grid(row=7, column=0)
    labella.grid(row=8, column=0)
    labello.grid(row=9, column=0)
    labelat.grid(row=10, column=0)
    labelri.grid(row=11, column=0)
    labelsn.grid(row=12, column=0)
    labelps.grid(row=13, column=0)
    labelpr.grid(row=14, column=0)
    
    labelb.grid(row=15, column=0)
    #labelbl.grid(row=16, column=0)
    #
    ITvL.grid(row=0, column=1)
    ETvL.grid(row=1, column=1)
    ptvL.grid(row=2, column=1)
    aXvL.grid(row=3, column=1)
    aYvL.grid(row=4, column=1)
    aZvL.grid(row=5, column=1)
    pevL.grid(row=6, column=1)
    tivL.grid(row=7, column=1)
    lavL.grid(row=8, column=1)
    lovL.grid(row=9, column=1)
    atvL.grid(row=10, column=1)
    rivL.grid(row=11, column=1)
    snvL.grid(row=12, column=1)
    psvL.grid(row=13, column=1)
    prvL.grid(row=14, column=1)
    

    headers = ['iTemp', 'eTemp', 'X', 'Y', 'Z', 'pressure', 'time', 'GPS coord', 'GPS coord', 'Alt','Pressure-Temperature','Packets Sent','Packets Recieved']
    if DEBUG_ON:
        print("opening csv")
    with open(csvFile, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        f.close()
    if DEBUG_ON:
        print("CSV closed")


# ***** GUI SETUP STOP *****


def loop():
    if DEBUG_ON:
        print("start loop")
    global start_flag
    global payload
    global sensor_data
    global string_rec
    global found_gga

    # ***** START OF PARSE SCRIPT *****
    while 1:
        test = 0
        while test != '[':
            try:
                test = ser.read().decode("utf-8")
                # if test == '[':
                #     print("YES")
                #     start_flag = True
            except:
                pass
            # print("couldn't decode character (this is okay)")  # This line can be used for debug if desired

        char = ser.read().decode("utf-8")
        string_rec = char
        while char != ']':
            char = ser.read().decode("utf-8")
            if char != ']' or '':
                string_rec = string_rec + char
        # print(string_rec)

        sensor_data[12] = sensor_data[12] + 1

        payload = np.asarray(string_rec.split(','))
        RSSI_SNR = payload[-2:]
        payload = payload[:-2].astype(np.int)
        if DEBUG_ON:
            print("RSSI_SNR")
            print(RSSI_SNR)
        # print(payload)
        if payload[0] != 91:  # Check for sensor data error and go back to start of loop
            if DEBUG_ON:
                print("ERROR: Sensor data invalid")
            continue
        payload_sensors = payload[1:21]
        payload_sensors[14] = 0
        if DEBUG_ON:
            print("payload")
            print(payload_sensors)
        payload_gps = np.asarray(("".join([chr(item) for item in payload[22:-2]])).split(','), np.str)
        if len(payload_gps) == 1:
            if DEBUG_ON:
                print("ERROR: GPS data invalid, try power cycle?")
            continue
        if DEBUG_ON:
            print("gps")
            print(payload_gps)

        # ARRAY TABLE #
        # 0 = internal temp | 1 = external temp | 2 = accel x | 3 = accel y | 4 = accel z | 5 = pressure
        sensor_data[0] = (payload_sensors[0] << 8) + payload_sensors[1]  # set internal temp
        sensor_data[1] = (payload_sensors[2] << 8) + payload_sensors[3]  # set external temp
        sensor_data[2] = (payload_sensors[4] << 8) + payload_sensors[5]  # set accel x
        sensor_data[3] = (payload_sensors[6] << 8) + payload_sensors[7]  # set accel y
        sensor_data[4] = (payload_sensors[8] << 8) + payload_sensors[9]  # set accel z
        sensor_data[5] = (payload_sensors[10] << 24) + (payload_sensors[11] << 16) + (payload_sensors[12] << 8) + \
                         payload_sensors[13]
        sensor_data[10] = (payload_sensors[14] << 24) + (payload_sensors[15] << 16) + (payload_sensors[16] << 8) + \
                          payload_sensors[17]
        sensor_data[11] = (payload_sensors[18] << 8) + payload_sensors[19]

        # ------- Convert Payload Items -------
        for i in range(5):  # loop through the first 5 values and assign them to the "payload"
            # this if statement handles if the number is negative (2s complement)
            if sensor_data[i].astype(int) >> 15:
                temp_list = list(bin(sensor_data[i].astype(int)))  # this is needed to do binary 2s complement
                for j in range(len(temp_list) - 2):
                    # Flip bits
                    if temp_list[j + 2] == '1':
                        temp_list[j + 2] = '0'
                    else:
                        temp_list[j + 2] = '1'
                temp_list = "".join(temp_list)  # join the list back together
                sensor_data[i] = ~ int(temp_list, 2)  # convert the temp list back to a number
            if i < 2:
                sensor_data[i] = sensor_data[i] / (2 ** 7)  # convert temperature
            if 1 < i < 5:
                sensor_data[i] = sensor_data[i] * 0.061  # convert accelerometer
        sensor_data[5] = sensor_data[5] / 100
        sensor_data[10] = sensor_data[10] / 100

        # ------- STOP Convert Payload Items -------
        if DEBUG_ON:
            print("Internal Temp:   " + str(sensor_data[0]))
            print("External Temp:   " + str(sensor_data[1]))
            print("X Accel:     " + str(sensor_data[2]))
            print("Y Accel:     " + str(sensor_data[3]))
            print("Z Accel:     " + str(sensor_data[4]))
            print("Pressure:    " + str(sensor_data[5]))
            print("Pressure Temp: " + str(sensor_data[10]))
            print("Packets sent: " + str(sensor_data[11]))
            print("Packets received " + str(sensor_data[12]))

        # ARRAY TABLE #
        # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
        gga_ind = np.where(payload_gps == "$GNGGA")  # find the index of GNGGA
        if len(gga_ind[0]) != 0:
            # print("GGA: " + gga_ind.astype(str))
            try:
                sensor_data[6] = (payload_gps[gga_ind[0][0] + 1].astype(np.float)).astype(np.int64)
                found_gga = True
            except ValueError:
                found_gga = False
                if DEBUG_ON:
                    print("Could not get GPS time, setting to 0")
        else:  # if we don't get this string, do some searching to try to get it another way
            for i in range(len(payload_gps)):
                if "$GNGGA" in payload_gps[i]:
                    # print("STR: " + payload_gps[i])
                    if i != len(payload_gps) - 1:
                        try:
                            sensor_data[6] = (payload_gps[i + 1].astype(np.float)).astype(np.int64)
                            found_gga = True
                            # print("alt GGA: " + str(gga_ind))
                            break
                        except ValueError:
                            pass
        if found_gga:
            if DEBUG_ON:
                print("Time:        " + str(sensor_data[6]))
            found_gga = False
        else:
            if DEBUG_ON:
                print("DID NOT GET TIME, SETTING TO 00:00:00")
            sensor_data[6] = 0

        # ARRAY TABLE #
        # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
        NS_ind = np.where(payload_gps == "N")
        if len(NS_ind[0]) != 0:
            if NS_ind[0][0] != 0:
                for i in range(len(NS_ind[0])):
                    try:
                        sensor_data[7] = (payload_gps[NS_ind[0][i] - 1].astype(np.float)) / 100
                        temp_dec = sensor_data[7] % 1
                        sensor_data[7] -= temp_dec
                        temp_dec = (temp_dec * 100) / 60
                        sensor_data[7] += temp_dec
                        if DEBUG_ON:
                            print("N Coord:     " + str(sensor_data[7]))
                    except ValueError:
                        if i == len(NS_ind[0]) - 1:
                            if DEBUG_ON:
                                print("N coordinate no valid index, set to 0")
                            sensor_data[7] = 0
                        pass
            else:
                if DEBUG_ON:
                    print("N coordinate out of index, set to 0")
                sensor_data[7] = 0
        else:
            NS_ind = np.where(payload_gps == "S")
            if len(NS_ind[0]) != 0:
                if NS_ind[0][0] != 0:
                    for i in range(len(NS_ind[0])):
                        try:
                            # set negative if in S hemisphere
                            sensor_data[7] = (payload_gps[NS_ind[0][i] - 1].astype(np.float)) / 100
                            temp_dec = sensor_data[7] % 1
                            sensor_data[7] -= temp_dec
                            temp_dec = (temp_dec * 100) / 60
                            sensor_data[7] += temp_dec
                            sensor_data[7] = sensor_data[7] * -1
                            if DEBUG_ON:
                                print("S Coord:     " + str(sensor_data[7]))
                        except ValueError:
                            if i == len(NS_ind[0]) - 1:
                                if DEBUG_ON:
                                    print("S coordinate no valid index, set to 0")
                                sensor_data[7] = 0
                            pass
                else:
                    if DEBUG_ON:
                        print("S coordinate out of index, set to 0")
                    sensor_data[7] = 0
            else:
                if DEBUG_ON:
                    print("NS coordinate not found in packet, set to 0")
                sensor_data[7] = 0

        # ARRAY TABLE #
        # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
        EW_ind = np.where(payload_gps == "E")
        if len(EW_ind[0]) != 0:
            if EW_ind[0][0] != 0:
                for i in range(len(EW_ind[0])):
                    try:
                        sensor_data[8] = (payload_gps[EW_ind[0][i] - 1].astype(np.float)) / 100
                        temp_dec = sensor_data[8] % 1
                        sensor_data[8] -= temp_dec
                        temp_dec = (temp_dec * 100) / 60
                        sensor_data[8] += temp_dec
                        if DEBUG_ON:
                            print("E Coord:     " + str(sensor_data[8]))
                    except ValueError:
                        if i == len(EW_ind[0]) - 1:
                            if DEBUG_ON:
                                print("E coordinate no valid index, set to 0")
                            sensor_data[8] = 0
                        pass
            else:
                if DEBUG_ON:
                    print("E coordinate out of index, set to 0")
                sensor_data[8] = 0
        else:
            EW_ind = np.where(payload_gps == "W")
            if len(EW_ind[0]) != 0:
                if EW_ind[0][0] != 0:
                    for i in range(len(EW_ind[0])):
                        try:
                            # set negative if in S hemisphere
                            sensor_data[8] = (payload_gps[EW_ind[0][i] - 1].astype(np.float)) / 100
                            temp_dec = sensor_data[8] % 1
                            sensor_data[8] -= temp_dec
                            temp_dec = (temp_dec * 100) / 60
                            sensor_data[8] += temp_dec
                            sensor_data[8] = sensor_data[8] * -1
                            if DEBUG_ON:
                                print("W Coord:     " + str(sensor_data[8]))
                        except ValueError:
                            if i == len(EW_ind[0]) - 1:
                                if DEBUG_ON:
                                    print("W coordinate no valid index, set to 0")
                                sensor_data[8] = 0
                            pass
                else:
                    if DEBUG_ON:
                        print("W coordinate out of index, set to 0")
                    sensor_data[8] = 0
            else:
                if DEBUG_ON:
                    print("EW coordinate not found in packet, set to 0")
                sensor_data[8] = 0

        # ARRAY TABLE #
        # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
        M_ind = np.where(payload_gps == "M")
        if len(M_ind[0]) != 0:
            if M_ind[0][0] != 0:
                try:
                    height_M = (payload_gps[M_ind[0][0] - 1].astype(np.float)).astype(np.int64)
                except ValueError:
                    height_M = sensor_data[9]
                if height_M < 0:
                    try:
                        height_M = (payload_gps[M_ind[0][1] - 1].astype(np.float)).astype(np.int64)
                    except ValueError:
                        height_M = sensor_data[9]
                    if height_M < 0:
                        height_M = 0
                        if DEBUG_ON:
                            print("array value not in data packet, set to 0")
            else:
                height_M = 0
                if DEBUG_ON:
                    print("array value out of bounds, set to 0")
        else:
            height_M = 0
            if DEBUG_ON:
                print("No height variable found, set to 0")
        sensor_data[9] = height_M
        if DEBUG_ON:
            print("Height:      " + str(sensor_data[9]))
            print()
    # ***** STOP OF PARSE SCRIPT *****

        # ***** START OF GUI SCRIPT *****
        time = list((sensor_data[6].astype(np.int)).astype(np.str))
        if len(time) == 5:
            time.insert(0, '0')
        elif len(time) == 4:
            time.insert(0, '0')
            time.insert(0, '0')
        elif len(time) != 6:
            if DEBUG_ON:
                print("TIME ERROR")

        ITvL.config(text=np.around(sensor_data[0], 4),font=("Helvetica",16))
        ETvL.config(text=np.around(sensor_data[1], 4),font=("Helvetica",16))
        ptvL.config(text=np.around(sensor_data[10], 4),font=("Helvetica",16))
        aXvL.config(text=np.around(sensor_data[2], 4),font=("Helvetica",16))
        aYvL.config(text=np.around(sensor_data[3], 4),font=("Helvetica",16))
        aZvL.config(text=np.around(sensor_data[4], 4),font=("Helvetica",16))
        pevL.config(text=np.around(sensor_data[5], 4),font=("Helvetica",16))
        try:
            tivL.config(text=(time[0] + time[1] + ":" + time[2] + time[3] + ":" + time[4] + time[5]),font=("Helvetica",16))
        except IndexError:
            tivL.config(text="00:00:00",font=("Helvetica", 16))
        lavL.config(text=np.around(sensor_data[7], 4),font=("Helvetica",16))
        lovL.config(text=np.around(sensor_data[8], 4),font=("Helvetica",16))
        atvL.config(text=np.around(sensor_data[9], 4),font=("Helvetica",16))
        rivL.config(text=RSSI_SNR[0],font=("Helvetica",16))
        snvL.config(text=RSSI_SNR[1], font=("Helvetica", 16))
        psvL.config(text=np.around(sensor_data[11], 4),font=("Helvetica",16))
        prvL.config(text=np.around(sensor_data[12], 4),font=("Helvetica",16))

        # read to csv and save
        with open(csvFile, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(sensor_data)
            f.close()
        # update gui with new values
        if DEBUG_ON:
            print("values updated")
        window.update()
        # window.after(10)
        # ***** STOP OF GUI SCRIPT *****


windowSetup()
loop()
window.mainloop()


