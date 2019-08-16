#!/usr/bin/python

print('Waiting for WIFI')
import serial
import time

from socketIO_client import SocketIO, LoggingNamespace

batteryLevel = '0'
count = 0

ser = serial.Serial('/dev/ttyACM0',9600)

def message_handler(mesg):
    global batteryLevel
    global count
    velocity = mesg.decode('utf-8')
    velocities = unicode.encode(velocity)
    ser.write(velocities)
    print(velocities)
    
	
    if count >=250:
        count = 0
        batteryLevel = ser.readline()
        socketIO.emit('battery',batteryLevel)
        print(batteryLevel)
    count = count+1
    print count

	

def on_connect():
    print('connected')
    

def on_reconnect():
    print('reconnect')

socketIO = SocketIO('192.168.0.101',5001,LoggingNamespace)

socketIO.on('connect', on_connect)
socketIO.on('velocity', message_handler)
socketIO.wait()


