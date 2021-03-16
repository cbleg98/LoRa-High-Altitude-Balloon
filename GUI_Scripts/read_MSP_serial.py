"""Provides a way to read the data send via LoRa to the Adafruit M0 Featherboard

This script will handle the intake and parsing of the hex (char) values to
then display them nicely on any chosen GUI. If you are going to make your own 
GUI instead of using the existing, start here!

There is a current method for getting the data at the top of the loop. There is
an additional commented method below if it comes in handy if there is a different
type of UART system (this originally worked with UART directly from the MSP430)
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.basemap import Basemap  # unused

__author__ = "Cameron Blegen"
__copyright__ = "Copyright, Montana Space Grant Consortium"
__version__ = "1.0.1"

# Globals
start_flag = False  # starts the while loop when we get start character
found_gga = False  # used for getting the time (due to parse errors)
i = 0
string_rec = ""
payload = np.zeros(500)
sensor_data = np.zeros(10)  # 0 = internal temp | 1 = external temp | 2 = accel x | 3 = accel y | 4 = accel z
                            # | 5 = pressure | 6 = time | 7 = GPS coords | 8 = GPS coords | 9 = height in m
# NOTE, the gps coordinate format is converted from DDM to DD. Coordinates are given in decimal degrees.

np.set_printoptions(suppress=True)
ser = serial.Serial('COM7')  # go to device manager and select the COM port you want
print(ser.name)  # print to make sure we have the right device

# plt.figure(figsize=(8, 8))  # unused
# m = Basemap(projection='ortho', resolution=None, lat_0=50, lon_0=-100)  # unused
# m.bluemarble(scale=0.5)  #unused

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

    payload = np.asarray(string_rec.split(','))
    payload = payload[:-1].astype(np.int)
    # print(payload)
    if payload[0] != 91:  # Check for sensor data error and go back to start of loop
        print("ERROR: Sensor data invalid")
        continue
    payload_sensors = payload[1:15]
    # print(payload_sensors)
    payload_gps = np.asarray(("".join([chr(item) for item in payload[16:]])).split(','), np.str)
    if len(payload_gps) == 1:
        print("ERROR: GPS data invalid, try power cycle?")
        continue
    # print(payload_gps)

    # ARRAY TABLE #
    # 0 = internal temp | 1 = external temp | 2 = accel x | 3 = accel y | 4 = accel z | 5 = pressure
    sensor_data[0] = (payload_sensors[0] << 8) + payload_sensors[1]  # set internal temp
    sensor_data[1] = (payload_sensors[2] << 8) + payload_sensors[3]  # set external temp
    sensor_data[2] = (payload_sensors[4] << 8) + payload_sensors[5]  # set accel x
    sensor_data[3] = (payload_sensors[6] << 8) + payload_sensors[7]  # set accel y
    sensor_data[4] = (payload_sensors[8] << 8) + payload_sensors[9]  # set accel z
    sensor_data[5] = (payload_sensors[10] << 24) + (payload_sensors[11] << 16) + (payload_sensors[12] << 8) + payload_sensors[13]

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

    # ------- STOP Convert Payload Items -------

    print("Internal Temp:   " + str(sensor_data[0]))
    print("External Temp:   " + str(sensor_data[1]))
    print("X Accel:     " + str(sensor_data[2]))
    print("Y Accel:     " + str(sensor_data[3]))
    print("Z Accel:     " + str(sensor_data[4]))
    print("Pressure:    " + str(sensor_data[5]))

    # START OF GPS #

    # ARRAY TABLE #
    # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
    gga_ind = np.where(payload_gps == "$GNGGA")  # find the index of GNGGA
    if len(gga_ind[0]) != 0:
        # print("GGA: " + gga_ind.astype(str))
        sensor_data[6] = (payload_gps[gga_ind[0][0] + 1].astype(np.float)).astype(np.int64)
        found_gga = True
    else:  # if we don't get this string, do some searching to try to get it another way
        for i in range(len(payload_gps)):
            if "$GNGGA" in payload_gps[i]:
                # print("STR: " + payload_gps[i])
                if i != len(payload_gps)-1:
                    try:
                        sensor_data[6] = (payload_gps[i+1].astype(np.float)).astype(np.int64)
                        found_gga = True
                        # print("alt GGA: " + str(gga_ind))
                        break
                    except ValueError:
                        pass
    if found_gga:
        print("Time:        " + str(sensor_data[6]))
        found_gga = False
    else:
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
                    print("N Coord:     " + str(sensor_data[7]))
                except ValueError:
                    if i == len(NS_ind[0]) - 1:
                        print("N coordinate no valid index, set to 2")
                        sensor_data[7] = 2
                    pass
        else:
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
                        print("S Coord:     " + str(sensor_data[7]))
                    except ValueError:
                        if i == len(NS_ind[0]) - 1:
                            print("S coordinate no valid index, set to 2")
                            sensor_data[7] = 2
                        pass
            else:
                print("S coordinate out of index, set to 0")
                sensor_data[7] = 0
        else:
            print("NS coordinate not found in packet, set to 1")
            sensor_data[7] = 1

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
                    print("E Coord:     " + str(sensor_data[8]))
                except ValueError:
                    if i == len(EW_ind[0]) - 1:
                        print("E coordinate no valid index, set to 2")
                        sensor_data[8] = 2
                    pass
        else:
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
                        print("W Coord:     " + str(sensor_data[8]))
                    except ValueError:
                        if i == len(EW_ind[0])-1:
                            print("W coordinate no valid index, set to 2")
                            sensor_data[8] = 2
                        pass
            else:
                print("W coordinate out of index, set to 0")
                sensor_data[8] = 0
        else:
            print("EW coordinate not found in packet, set to 1")
            sensor_data[8] = 1

    # ARRAY TABLE #
    # 6 = time | 7 = GPS coords N/S | 8 = GPS coords E/W | 9 = height in m
    M_ind = np.where(payload_gps == "M")
    if len(M_ind[0]) != 0:
        if M_ind[0][0] != 0:
            try:
                height_M = (payload_gps[M_ind[0][0]-1].astype(np.float)).astype(np.int64)
            except ValueError:
                print("array value not in data packet, set to 0")
                height_M = 0
            if height_M < 0:
                try:
                    height_M = (payload_gps[M_ind[0][1]-1].astype(np.float)).astype(np.int64)
                except ValueError:
                    print("array value not in data packet, set to 0")
                    height_M = 0
                if height_M < 0:
                    height_M = 0
                    print("array value not in data packet, set to 0")
        else:
            height_M = 0
            print("array value out of bounds, set to 0")
    else:
        height_M = 0
        print("No height variable found, set to 0")
    sensor_data[9] = height_M
    print("Height:      " + str(sensor_data[7]))
    print()

    # OLD METHOD FOR RECEIVING DATA, MAY BE USEFUL IF USING A DIFFERENT SETUP
        # ex = ser.read(14)  # read the numerical values from the serial port
        # a = list(ex)  # turn bytes into a list
        #
        # # ------- GET SENSOR VALUES AND PUT IN PAYLOAD -------
        # for i in range(5):  # loop through the first 5 values and assign them to the "payload"
        #     # get the two byte value and assign
        #     payload[i] = a[i+i] << 8
        #     payload[i] += a[i+i+1]
        #
        #     # this if statement handles if the number is negative (2s complement)
        #     if payload[i].astype(int) >> 15:
        #         temp_list = list(bin(payload[i].astype(int)))  # this is needed to do binary 2s complement
        #         for j in range(len(temp_list) - 2):
        #             # Flip bits
        #             if temp_list[j+2] == '1':
        #                 temp_list[j+2] = '0'
        #             else:
        #                 temp_list[j+2] = '1'
        #         temp_list = "".join(temp_list)  # join the list back together
        #         payload[i] = ~ int(temp_list, 2)  # convert the temp list back to a number
        #     if i < 2:
        #         payload[i] = payload[i]/(2**7)  # convert temperature
        #     if 1 < i < 5:
        #         payload[i] = payload[i]*0.061  # convert accelerometer
        # payload[5] = (a[10] << 24) + (a[11] << 16) + (a[12] << 8) + a[13]
        # payload[5] = payload[5]/100
        #
        # # ------- GET GPS VALUES AND PUT IN PAYLOAD -------
        # ex = ser.read(117)
        # split_gps = str(ex).split(',')
        #
        # payload[6] = split_gps[1]  # Get the time
        #
        # # Get the N/S coordinate and convert to Decimal Degree form
        # payload[7] = float(split_gps[2])/100
        # temp_dec = payload[7] % 1
        # payload[7] -= temp_dec
        # temp_dec = (temp_dec*100)/60
        # payload[7] += temp_dec
        # if split_gps[3] == 'S':  # Check what N/S hemisphere we are in
        #     payload[7] = payload[7] * -1  # Set the right negative value if in S hemisphere
        #
        # # Get the E/W coordinate and convert to Decimal Degree form
        # payload[8] = float(split_gps[4])/100
        # temp_dec = payload[8] % 1
        # payload[8] -= temp_dec
        # temp_dec = (temp_dec*100)/60
        # payload[8] += temp_dec
        # if split_gps[5] == 'W':  # Check what W/E hemisphere we are in
        #     payload[8] = payload[8] * -1  # Set the right negative value if in W hemisphere
        #
        # payload[9] = split_gps[9]  # Get height of board in m
        #
        # print(payload)
        # print(split_gps)

        # PUT CODE HERE
        # ACCESS EACH INDEX OF THE "payload" ARRAY (SEE COMMENT AT TOP OF FILE FOR ORDER)
        # THEN READ IT OUT TO YOUR GUI
        # YOU CAN UPDATE THE VALUE ON YOUR GUI EACH TIME THROUGH THE LOOP
        # STARTING BY DISPLAYING THE GIVEN SAMPLE DATA WOULD BE GREAT
        # YOU GOT THIS!

        # print("you got this!")
