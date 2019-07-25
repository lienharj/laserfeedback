import serial
import time
import csv
import datetime
import matplotlib.pyplot as plt
from drawnow import *

port_var = 'com7'
baudrate_var = 230400
timeout_var = 0.2
data1 = []
data2 = []

def getValues(ser): # reads one line of the serial port, decodes it with ascii and splits the values, used in other the functions
    serData = ser.readline().decode('ascii').replace('\r\n', '').split(' ')
    return serData

def saveValues(N, name, title, structure):  # saves values in a .csv file
    ser = serial.Serial(port_var, baudrate=baudrate_var, timeout=timeout_var)
    time.sleep(2)
    start = datetime.datetime.now()
    print(start)
    titlelist = [title, start.strftime("%y%m%d %H:%M"), ""]
    struc_list = structure.split(' ')
    getValues(ser)
    with open("data/"+ name + ".csv", 'w') as output:
        writer = csv.writer(output, lineterminator='\n')
        writer.writerow(titlelist)
        writer.writerow(struc_list)
        i = 0
        while i < N:
            i += 1
            value = getValues(ser)
            print(value)
            writer.writerow(value)
    ser.close()
    stop = datetime.datetime.now()
    print(stop)
    duration = (stop-start).total_seconds()
    print("Duration: "+ str(duration)+"s")
    print("Data successfully saved")
# Example for use: saveValues(2000,"test01","Test01","Input Output Error")
# Arguments:    number of value sets to save, file name, infile name, structure of value sets

def makeFig():  # creates a figure, used in other the plotting functions
    plt.ylim(0,5000)
    plt.grid(True)
    plt.plot(data1, 'ro-', label='data1')
    plt.legend(loc='upper left')

    plt2 = plt.twinx()
    plt.ylim(0,5000)
    plt2.plot(data2, 'b^-', label='data2')
    plt2.ticklabel_format(useOffset=False)
    plt2.legend(loc='upper right')

def plotsaveValues(N, name, title, structure, m, n):    # plots the m and n value of the reveived value sets and also saves them in a .csv file
    ser = serial.Serial(port_var, baudrate=baudrate_var, timeout=timeout_var)
    time.sleep(2)
    cnt = 0
    plt.ion()
    now = datetime.datetime.now()
    print(now)
    titlelist = [title, now.strftime("%y%m%d %H:%M"), ""]
    struc_list = structure.split(' ')
    getValues(ser)
    with open(name + ".csv", 'w') as output:
        writer = csv.writer(output, lineterminator='\n')
        writer.writerow(titlelist)
        writer.writerow(struc_list)
        i = 0
        while i < N:
            i += 1
            value = getValues(ser)
            print(value)
            writer.writerow(value)
            data1.append(float(value[m]))
            data2.append(float(value[n]))
            drawnow(makeFig)
            plt.pause(.000001)
            cnt = cnt + 1
            if cnt > 50:
                data1.pop(0)
                data2.pop(0)
    ser.close()
    now = datetime.datetime.now()
    print(now)
    print("Data successfully saved")
# Example for use: plotsaveValues(2000,"test01","Test01","Input Output Error",1,2)
# Arguments:    number of value sets to save, file name, infile name, structure of value sets, value to plot 1, value to plot 2

def plotValues(N,m,n):  # plots the m and n value of the reveived value sets.
    ser = serial.Serial(port_var, baudrate=baudrate_var, timeout=timeout_var)
    time.sleep(2)
    cnt = 0
    plt.ion()
    now = datetime.datetime.now()
    print(now)
    i = 0
    getValues(ser)
    while i < N:
        i += 1
        value = getValues(ser)
        print(value)
        data1.append(float(value[m]))
        data2.append(float(value[n]))
        drawnow(makeFig)
        plt.pause(.001)
        cnt = cnt + 1
        if cnt > 100:
            data1.pop(0)
            data2.pop(0)
    ser.close()
    now = datetime.datetime.now()
    print(now)
    print("Data plotted")
    input("Press Enter to close the figure.")
# Example for use: plotValues(2000,1,2)
# Arguments:    number of value sets to save, value to plot 1, value to plot 2
