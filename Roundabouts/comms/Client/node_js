#!/usr/bin/node

var serialport = require('serialport');
var SerialPort = serialport.SerialPort;
var port = new SerialPort('/dev/ttyACM0',{
	baudRate:9600,
	parser:serialport.parsers.readline('\r\n')
});

var io = require('socket.io-client')
var socket = io.connect('http://192.168.0.101:5001');
var batteryLevel;
var count = 0;

port.on('open', function(){
	console.log('COM on open')
});

port.on('data',function(data){
	batteryLevel = data;
});
	
console.log('waiting for WIFI')


socket.on('connect',function(){
	console.log('connected')
});
socket.on('velocity',function(data){
	count = count + 1
    console.log(data)
    console.log(count)
    port.write(data)
});
 
setInterval(function(){
	socket.emit('battery',batteryLevel)
	console.log('batteryLevel:'+batteryLevel)
},4000);

