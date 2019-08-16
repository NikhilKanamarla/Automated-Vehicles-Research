#!/usr/bin/python
from socketIO_client import SocketIO, LoggingNamespace
import rospy
from std_msgs.msg import String

print('Waiting for WIFI')

batteryLevel = '0'
batt11 = 0
batt12 = 0
batt13 = 0
batt14 = 0
batt15 = 0
batt6 = 0
batt7 = 0
batt8 = 0
batt9 = 0
batt10 = 0
battLev = 0
index = 0

def talker(battLev):
    global pub
    pub.publish(str(index)+","+str(battLev))

pub = rospy.Publisher('/ZumoBatts', String, queue_size=1000)

def message_handler(args):
    global batt6
    global batt7
    global batt8
    global batt9
    global batt10
    global batt11
    global batt12
    global batt13
    global batt14
    global batt15
    global batt16
    global batt17
    global batt18
    global batt19
    global batt20
    global batt21
    global batt22
    global batt23
    global batt24
    global batt25
    global index
 
    if args[:2] == '06': #this is the IP address (the first 2 elements of data)
	batt6 = args[2:] #batt6 is 7200 for example
        index = 6
        print ("Getting robot "+ str(6)+ " batt: " + str(batt6))
        talker(batt6)
    if args[:2] == '07':
	batt7 = args[2:]
        index = 7
        print ("Getting robot "+ str(7)+ " batt: " + str(batt7))
        talker(batt7)
    if args[:2] == '08':
	batt8 = args[2:]
        index = 8
        print ("Getting robot "+ str(8)+ " batt: " + str(batt8))
        talker(batt8)
    if args[:2] == '09':
	batt9 = args[2:]
        index = 9
        print ("Getting robot "+ str(9)+ " batt: " + str(batt9))
        talker(batt9)
    if args[:2] == '10':
	batt10 = args[2:]
        index = 10
        print ("Getting robot "+ str(10)+ " batt: " + str(batt10))
        talker(batt10)
    if args[:2] == '11':
	batt11 = args[2:]
        index = 11
        print ("Getting robot "+ str(11)+ " batt: " + str(batt11))
        talker(batt11)
    if args[:2] == '12':
	batt12 = args[2:]
        index = 12
        print ("Getting robot "+ str(12)+ " batt: " + str(batt12))
        talker(batt12)
    if args[:2] == '13':
	batt13 = args[2:]
        index = 13
        print ("Getting robot "+ str(13)+ " batt: " + str(batt13))
        talker(batt13)
    if args[:2] == '14':
	batt14 = args[2:]
        index = 14
        print ("Getting robot "+ str(14)+ " batt: " + str(batt14))
        talker(batt14)
    if args[:2] == '15':
	batt15 = args[2:]
        index = 15
        print ("Getting robot "+ str(15)+ " batt: " + str(batt15))
        talker(batt15)
    if args[:2] == '13':
	batt13 = args[2:]
        index = 13
        print ("Getting robot "+ str(13)+ " batt: " + str(batt13))
        talker(batt13)
    if args[:2] == '13':
	batt13 = args[2:]
        index = 13
        print ("Getting robot "+ str(13)+ " batt: " + str(batt13))
        talker(batt13)
    if args[:2] == '25':
	batt25 = args[2:]
        index = 25
        print ("Getting robot "+ str(25)+ " batt: " + str(batt25))
        talker(batt25)
    print "waiting for batt update"


def on_connect():
    print('connected')

def on_reconnect():
    print('reconnect')

rospy.init_node('zumo_batt', anonymous=True)

socketIO = SocketIO('localhost',5001,LoggingNamespace)

socketIO.on('connect', on_connect)
socketIO.on('batt', message_handler)
socketIO.wait()
