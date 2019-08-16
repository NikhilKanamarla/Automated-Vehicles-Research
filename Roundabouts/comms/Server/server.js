
var io = require('socket.io')()

var socket_list = new Array();

io.on('connection',function(socket){
	//to connect
	var batt_update_time;

	var ip = socket.request.connection.remoteAddress;
	clientip = ip.substring(18);
	console.log("Socket " +"robot" +clientip + " connected !");
	
	
	//sent velocities
	setInterval(function(){
		rightVelocity = '0.5';
		leftVelocity = '0.5'
		socket.emit('velocity',rightVelocity+','+leftVelocity+';')
	},100);
	
	//recive battery level from clients
	socket.on('battery', function(data){		
			batt_update_time = new Date().getTime();
			console.log(clientip + " has battery level:" + data.toString());
	});
	//disconnect handler
	var if_connected = setInterval(function(){ 
			var now = new Date().getTime();
			var disconnect_tester = now - batt_update_time;
			if(disconnect_tester > 10000){
				console.log("Socket " +"robot" +clientip + " disconnected.");
				clearInterval(if_connected);			
			}
	}, 1000);
	
	
	
});

io.listen(5001);
console.log("Server Started");
//document.getElementById("conntag").innerHTML="dddd";
