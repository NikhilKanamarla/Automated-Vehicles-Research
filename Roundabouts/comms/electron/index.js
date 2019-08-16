const {app,BrowserWindow,ipcMain} = require('electron');
var io = require('socket.io')()
const path = require('path')
const url = require('url')


var rightVelocity1='0';
var leftVelocity1='0';
var rightVelocity2='0';
var leftVelocity2='0';
var rightVelocity3='0';
var leftVelocity3='0';
var rightVelocity4='0';
var leftVelocity4='0';
var rightVelocity5='0';
var leftVelocity5='0';
var rightVelocity6='0';
var leftVelocity6='0';
var rightVelocity7='0';
var leftVelocity7='0';
var rightVelocity8='0';
var leftVelocity8='0';
var rightVelocity9='0';
var leftVelocity9='0';
var rightVelocity10='0';
var leftVelocity10='0';
var rightVelocity11='0';
var leftVelocity11='0';
var rightVelocity12='0';
var leftVelocity12='0';
var rightVelocity13='0';
var leftVelocity13='0';
var rightVelocity14='0';
var leftVelocity14='0';
var rightVelocity15='0';
var leftVelocity15='0';
var rightVelocity16='0';
var leftVelocity16='0';
var rightVelocity17='0';
var leftVelocity17='0';
var rightVelocity18='0';
var leftVelocity18='0';
var rightVelocity19='0';
var leftVelocity19='0';
var rightVelocity20='0';
var leftVelocity20='0';
var rightVelocity21='0';
var leftVelocity21='0';
var rightVelocity22='0';
var leftVelocity22='0';
var rightVelocity23='0';
var leftVelocity23='0';
var rightVelocity24='0';
var leftVelocity24='0';
var rightVelocity25='0';
var leftVelocity25='0';
var rightVelocity26='0';
var leftVelocity26='0';
var rightVelocity27='0';
var leftVelocity27='0';
var rightVelocity28='0';
var leftVelocity28='0';
var rightVelocity29='0';
var leftVelocity29='0';
var rightVelocity30='0';
var leftVelocity30='0';
var rightVelocity31='0';
var leftVelocity31='0';
var rightVelocity32='0';
var leftVelocity32='0';
var rightVelocity33='0';
var leftVelocity33='0';
var rightVelocity34='0';
var leftVelocity34='0';
var rightVelocity35='0';
var leftVelocity35='0';

var count = 0;
var batt1='000000';
var batt2='000000';
var batt3='000000';
var batt4='000000';
var batt5='000000';
var batt6='000000';
var batt7='000000';
var batt8='000000';
var batt9='000000';
var batt10='000000';
var batt11='000000';
var batt12='000000';
var batt13='000000';
var batt14='000000';
var batt15='000000';
var batt16='000000';
var batt17='000000';
var batt18='000000';
var batt19='000000';
var batt20='000000';
var batt21='000000';
var batt22='000000';
var batt23='000000';
var batt24='000000';
var batt25='000000';
var batt26='000000';
var batt27='000000';
var batt28='000000';
var batt29='000000';
var batt30='000000';
var batt31='000000';
var batt32='000000';
var batt33='000000';
var batt34='000000';
var batt35='000000';
// modified by Sumeet
var rightVelocity = new Array(36);
var leftVelocity = new Array(36);
index = 0;

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.


function createWindow () {
  // Create the browser window.
  mainWindow = new BrowserWindow({width: 1920, height: 1080})

  // and load the index.html of the app.
  mainWindow.loadURL(url.format({
    pathname: path.join(__dirname, 'index.html'),
    protocol: 'file:',
    slashes: true
  }))

  // Open the DevTools.
  mainWindow.webContents.openDevTools()

  // Emitted when the window is closed.
  mainWindow.on('closed', function () {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    mainWindow = null
  })
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on('ready', createWindow)

// Quit when all windows are closed.
app.on('window-all-closed', function () {
  // On OS X it is common for applications and their menu bar
  // to stay active until the user quits explicitly with Cmd + Q
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', function () {


  if (mainWindow === null) {
    createWindow()
  }
})

  // On OS X it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  var socket_list = new Array();
  var win_sender;

  
  io.on('connection',function(socket){
    var batt_update_time;
    var batt_lev = '';

  //to connect
    var ip = socket.request.connection.remoteAddress;
    var clientip = ip.substring(18);

//    var emitter = require('socket.io-emitter')("localhost:5001");
//    emitter.redis.on('error',function(err){
//        console.log(err);
//    });

    //console.log("Socket" +"robot" +clientip + " connected !");

    win_sender.send('asynchronous-reply', 'connected', clientip)

  //sent velocities
 
    socket.on('getvelocity',function(velocities){
    count = count +1;
    console.log(count);
    // modified by Sumeet
	index = velocities[0].index;
	rightVelocity[index] = velocities[0].lv;
	leftVelocity[index] = velocities[0].rv;
	console.log("test point");
	console.log(rightVelocity[index]);                            
    })


    //recive battery level from clients
      socket.on('battery', function(data){    
          batt_update_time = new Date().getTime();
          //console.log('robot'+ clientip + " has battery level:" + data.toString());
          batt_lev = batt_lev + data.toString().substring(0,4)+'\t'
          win_sender.send('battery_level',data.toString(),clientip)
	  if (clientip == '01'){
		batt1 = clientip + data.toString()
             }
	  if (clientip == '02'){
		batt2 = clientip + data.toString()
             }
	  if (clientip == '03'){
		batt3 = clientip + data.toString()
             }
	  if (clientip == '04'){
		batt4 = clientip + data.toString()
             }
	  if (clientip == '05'){
		batt5 = clientip + data.toString()
             }
	  if (clientip == '06'){
		batt6 = clientip + data.toString()
             }
	  if (clientip == '07'){
		batt7 = clientip + data.toString()
             }
	  if (clientip == '08'){
		batt8 = clientip + data.toString()
             }
	  if (clientip == '09'){
		batt9 = clientip + data.toString()
             }
	  if (clientip == '10'){
		batt10 = clientip + data.toString()
             }
	  if (clientip == '11'){
		batt11 = clientip + data.toString()
             }
	  if (clientip == '12'){
		batt12 = clientip + data.toString()
             }
	  if (clientip == '13'){
		batt13 = clientip + data.toString()
             }
	  if (clientip == '14'){
		batt14 = clientip + data.toString()
             }
	  if (clientip == '15'){
		batt15 = clientip + data.toString()
             }
	  if (clientip == '16'){
		batt16 = clientip + data.toString()
             }
	  if (clientip == '17'){
		batt17 = clientip + data.toString()
             }
	  if (clientip == '18'){
		batt18 = clientip + data.toString()
             }
	  if (clientip == '19'){
		batt19 = clientip + data.toString()
             }
	  if (clientip == '20'){
		batt20 = clientip + data.toString()
             }
    if (clientip == '21'){
    batt21 = clientip + data.toString()
             }
    if (clientip == '22'){
    batt22 = clientip + data.toString()
             }
    if (clientip == '23'){
    batt23 = clientip + data.toString()
             }
    if (clientip == '24'){
    batt24 = clientip + data.toString()
             }
    if (clientip == '25'){
    batt25 = clientip + data.toString()
             }
    if (clientip == '26'){
    batt26 = clientip + data.toString()
             }
    if (clientip == '27'){
    batt27 = clientip + data.toString()
             }
    if (clientip == '28'){
    batt28 = clientip + data.toString()
             }
    if (clientip == '29'){
    batt29 = clientip + data.toString()
             }
    if (clientip == '30'){
    batt30 = clientip + data.toString()
             } 
    if (clientip == '31'){
    batt31 = clientip + data.toString()
             }
    if (clientip == '32'){
    batt32 = clientip + data.toString()
             }
    if (clientip == '33'){
    batt33 = clientip + data.toString()
             }
    if (clientip == '34'){
    batt34 = clientip + data.toString()
             }
    if (clientip == '35'){
    batt35 = clientip + data.toString()
             }
    
   });


  //disconnect handler
    var if_connected = setInterval(function(){ 
        var now = new Date().getTime();
        var disconnect_tester = now - batt_update_time;
        if(disconnect_tester > 30000){
          console.log("Socket " +"robot" +clientip + " disconnected.");
          win_sender.send('asynchronous-reply', 'Disconnect', clientip) 
          console.log('Robot'+clientip+":",batt_lev)
          clearInterval(if_connected);
          socket.disconnect(true)
          if (clientip == '01'){
              rightVelocity1='0';
              leftVelocity1='0';
            }
          if (clientip == '02'){
              rightVelocity2='0';
              leftVelocity2='0';
            }          
          if (clientip == '03'){
              rightVelocity3='0';
              leftVelocity3='0';
            }
          if (clientip == '04'){
              rightVelocity4='0';
              leftVelocity4='0';
            }
          if (clientip == '05'){
              rightVelocity5='0';
              leftVelocity5='0';
            }
          if (clientip == '06'){
              rightVelocity6='0';
              leftVelocity6='0';
            }
          if (clientip == '07'){
              rightVelocity7='0';
              leftVelocity7='0';
            }          
          if (clientip == '08'){
              rightVelocity8='0';
              leftVelocity8='0';
            }
          if (clientip == '09'){
              rightVelocity9='0';
              leftVelocity9='0';
            }
          if (clientip == '10'){
              rightVelocity10='0';
              leftVelocity10='0';
            }  
          if (clientip == '11'){
              rightVelocity11='0';
              leftVelocity11='0';
            } 
          if (clientip == '12'){
              rightVelocity12='0';
              leftVelocity12='0';
            } 
          if (clientip == '13'){
              rightVelocity13='0';
              leftVelocity13='0';
            } 
          if (clientip == '14'){
              rightVelocity14='0';
              leftVelocity14='0';
            } 
          if (clientip == '15'){
              rightVelocity15='0';
              leftVelocity15='0';
            } 
          if (clientip == '16'){
              rightVelocity16='0';
              leftVelocity16='0';
            } 
          if (clientip == '17'){
              rightVelocity17='0';
              leftVelocity17='0';
            } 
          if (clientip == '18'){
              rightVelocity18='0';
              leftVelocity18='0';
            } 
          if (clientip == '19'){
              rightVelocity19='0';
              leftVelocity19='0';
            }
          if (clientip == '20'){
              rightVelocity20='0';
              leftVelocity20='0';
            }    
          if (clientip == '21'){
              rightVelocity21='0';
              leftVelocity21='0';
            } 
          if (clientip == '22'){
              rightVelocity22='0';
              leftVelocity22='0';
            } 
          if (clientip == '23'){
              rightVelocity23='0';
              leftVelocity23='0';
            } 
          if (clientip == '24'){
              rightVelocity24='0';
              leftVelocity24='0';
            } 
          if (clientip == '25'){
              rightVelocity25='0';
              leftVelocity25='0';
            } 
          if (clientip == '26'){
              rightVelocity26='0';
              leftVelocity26='0';
            } 
          if (clientip == '27'){
              rightVelocity27='0';
              leftVelocity27='0';
            } 
          if (clientip == '28'){
              rightVelocity28='0';
              leftVelocity28='0';
            } 
          if (clientip == '29'){
              rightVelocity29='0';
              leftVelocity29='0';
            }
          if (clientip == '30'){
              rightVelocity30='0';
              leftVelocity30='0';
            } 
          if (clientip == '31'){
              rightVelocity31='0';
              leftVelocity31='0';
            } 
          if (clientip == '32'){
              rightVelocity32='0';
              leftVelocity32='0';
            } 
          if (clientip == '33'){
              rightVelocity33='0';
              leftVelocity33='0';
            } 
          if (clientip == '34'){
              rightVelocity34='0';
              leftVelocity34='0';
            } 
          if (clientip == '35'){
              rightVelocity35='0';
              leftVelocity35='0';
            }
                                   
        }
        // modified by Sumeet
       for(var z = 0; z<36;z++){
	socket.emit('velocity'+(z+1), (leftVelocity[z+1])+','+(rightVelocity[z+1])+';')   
//socket.emit('velocity'+((z+1).toString()), '1'+','+((rightVelocity[z+1]).toString())+';')   	
}
        //count = count + 1;
      /*  socket.emit('batt', batt1)
      	socket.emit('batt', batt2)
      	socket.emit('batt', batt3)
      	socket.emit('batt', batt4)
      	socket.emit('batt', batt5)
      	socket.emit('batt', batt6)
      	socket.emit('batt', batt7)
      	socket.emit('batt', batt8)
      	socket.emit('batt', batt9)
      	socket.emit('batt', batt10)
      	socket.emit('batt', batt11)
      	socket.emit('batt', batt12)
      	socket.emit('batt', batt13)
      	socket.emit('batt', batt14)
      	socket.emit('batt', batt15)
      	socket.emit('batt', batt16)
      	socket.emit('batt', batt17)
      	socket.emit('batt', batt18)
      	socket.emit('batt', batt19)
      	socket.emit('batt', batt10)
        socket.emit('batt', batt21)
        socket.emit('batt', batt22)
        socket.emit('batt', batt23)
        socket.emit('batt', batt24)
        socket.emit('batt', batt25)
        socket.emit('batt', batt26)
        socket.emit('batt', batt27)
        socket.emit('batt', batt28)
        socket.emit('batt', batt29)
        socket.emit('batt', batt30)
        socket.emit('batt', batt31)
        socket.emit('batt', batt32)
        socket.emit('batt', batt33)
        socket.emit('batt', batt34)
        socket.emit('batt', batt35)
        socket.emit('batt', batt36)
      	socket.emit('batt', batt37)
      	socket.emit('batt', batt38)
      	socket.emit('batt', batt39)
      	socket.emit('batt', batt40)
      	socket.emit('batt', batt41)
      	socket.emit('batt', batt42)
      	socket.emit('batt', batt43)
      	socket.emit('batt', batt44)
      	socket.emit('batt', batt45)
      	socket.emit('batt', batt46)
      	socket.emit('batt', batt47)
      	socket.emit('batt', batt48)
      	socket.emit('batt', batt49)
      	socket.emit('batt', batt50)
      	socket.emit('batt', batt51)
      	socket.emit('batt', batt52)
      	socket.emit('batt', batt53)
      	socket.emit('batt', batt54)
      	socket.emit('batt', batt55)
        socket.emit('batt', batt56)
        socket.emit('batt', batt57)
        socket.emit('batt', batt58)
        socket.emit('batt', batt59)
        socket.emit('batt', batt60)
*/
        //console.log('Robot1 send:'+rightVelocity1+","+leftVelocity1)
        //console.log('Robot2 send:'+rightVelocity2+","+leftVelocity2)
        //console.log('Robot3 send:'+rightVelocity3+","+leftVelocity3)
        //console.log('Robot4 send:'+rightVelocity4+","+leftVelocity4)
        //console.log('Robot5 send:'+rightVelocity5+","+leftVelocity5)
        //console.log('Robot6 send:'+rightVelocity6+","+leftVelocity6)
        //console.log('Robot7 send:'+rightVelocity7+","+leftVelocity7)
        //console.log('Robot8 send:'+rightVelocity8+","+leftVelocity8)
        //console.log('Robot9 send:'+rightVelocity9+","+leftVelocity9)
        //console.log('Robot10 send:'+rightVelocity10+","+leftVelocity10)
        //console.log('Robot11 send:'+rightVelocity11+","+leftVelocity11)
        //console.log('Robot12 send:'+rightVelocity12+","+leftVelocity12)
        //console.log('Robot13 send:'+rightVelocity13+","+leftVelocity13)
        //console.log('Robot14 send:'+rightVelocity14+","+leftVelocity14)
        //console.log('Robot15 send:'+rightVelocity15+","+leftVelocity15)
        //console.log('Robot16 send:'+rightVelocity16+","+leftVelocity16)
        //console.log('Robot17 send:'+rightVelocity17+","+leftVelocity17)
        //console.log('Robot18 send:'+rightVelocity18+","+leftVelocity18)
        //console.log('Robot19 send:'+rightVelocity19+","+leftVelocity19)
        //console.log('Robot27 send:'+rightVelocity27+","+leftVelocity27)
        //console.log(count)
          
    }, 70);
  
  }); 

ipcMain.on('asynchronous-message', (event, arg1, agr2) => {
   win_sender = event.sender;
})

  io.listen(5001);
  console.log("Server Started");

//setInterval(function(){ 
//    socket.emit('velocity', rightVelocity+','+leftVelocity+';')
//  },100)

  
