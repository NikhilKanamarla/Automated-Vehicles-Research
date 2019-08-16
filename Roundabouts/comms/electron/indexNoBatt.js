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
var count = 0;

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.


function createWindow () {
  // Create the browser window.
  mainWindow = new BrowserWindow({width: 800, height: 600})

  // and load the index.html of the app.
  mainWindow.loadURL(url.format({
    pathname: path.join(__dirname, 'index.html'),
    protocol: 'file:',
    slashes: true
  }))

  // Open the DevTools.
  // mainWindow.webContents.openDevTools()

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
    if (velocities.substr(0,2) == '1:'){
      rightVelocity1 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity1 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot1:'+rightVelocity1+","+leftVelocity1)
    }
    if (velocities.substr(0,2) == '2:'){
      rightVelocity2 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity2 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot2:'+rightVelocity2+","+leftVelocity2)
    }
    if (velocities.substr(0,2) == '3:'){
      rightVelocity3 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity3 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot3:'+rightVelocity3+","+leftVelocity3)
    }
    if (velocities.substr(0,2) == '4:'){
      rightVelocity4 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity4 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot4:'+rightVelocity4+","+leftVelocity4)
    }
    if (velocities.substr(0,2) == '5:'){
      rightVelocity5 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity5 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot5:'+rightVelocity5+","+leftVelocity5)
    }
    if (velocities.substr(0,2) == '6:'){
      rightVelocity6 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity6 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot6:'+rightVelocity6+","+leftVelocity6)
    }
    if (velocities.substr(0,2) == '7:'){
      rightVelocity7 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity7 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot7:'+rightVelocity7+","+leftVelocity7)
    }
    if (velocities.substr(0,2) == '8:'){
      rightVelocity8 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity8 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot8:'+rightVelocity8+","+leftVelocity8)
    }
    if (velocities.substr(0,2) == '9:'){
      rightVelocity9 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity9 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot9:'+rightVelocity9+","+leftVelocity9)
    }
    if (velocities.substr(0,2) == '10'){
      rightVelocity10 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity10 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot10:'+rightVelocity10+","+leftVelocity10)
    }
    if (velocities.substr(0,2) == '11'){
      rightVelocity11 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity11 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot11:'+rightVelocity11+","+leftVelocity11)
    }     
    if (velocities.substr(0,2) == '12'){
      rightVelocity12 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity12 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot12:'+rightVelocity12+","+leftVelocity12)
    }
    if (velocities.substr(0,2) == '13'){
      rightVelocity13 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity13 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot10:'+rightVelocity13+","+leftVelocity13)
    }
    if (velocities.substr(0,2) == '14'){
      rightVelocity14 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity14 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot14:'+rightVelocity14+","+leftVelocity14)
    }
    if (velocities.substr(0,2) == '15'){
      rightVelocity15 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity15 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot15:'+rightVelocity15+","+leftVelocity15)
    }
    if (velocities.substr(0,2) == '16'){
      rightVelocity16 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity16 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot16:'+rightVelocity16+","+leftVelocity16)
    }
    if (velocities.substr(0,2) == '17'){
      rightVelocity17 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity17 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot17:'+rightVelocity17+","+leftVelocity17)
    }   
    if (velocities.substr(0,2) == '18'){
      rightVelocity18 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity18 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot18:'+rightVelocity18+","+leftVelocity18)
    }    
    if (velocities.substr(0,2) == '19'){
      rightVelocity19 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity19 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot19:'+rightVelocity19+","+leftVelocity19)
    }    
    if (velocities.substr(0,2) == '20'){
      rightVelocity20 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity20 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      console.log('Robot20:'+rightVelocity20+","+leftVelocity20)
    }     
    })


    //recive battery level from clients
      socket.on('battery', function(data){    
          batt_update_time = new Date().getTime();
          batt_lev = batt_lev + data.toString().substring(0,4)+'\t'
          win_sender.send('battery_level',data.toString(),clientip)
	 	
   });


  //disconnect handler
    var if_connected = setInterval(function(){ 
        var now = new Date().getTime();
        var disconnect_tester = now - batt_update_time;
        if(disconnect_tester > 10000){
          console.log("Socket " +"robot" +clientip + " disconnected.");
          win_sender.send('asynchronous-reply', 'Disconnect', clientip) 
          //console.log('Robot'+clientip+":",batt_lev)
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
        }
        socket.emit('velocity1', leftVelocity1+','+rightVelocity1+';')
        socket.emit('velocity2', leftVelocity2+','+rightVelocity2+';')
        socket.emit('velocity3', leftVelocity3+','+rightVelocity3+';')
        socket.emit('velocity4', leftVelocity4+','+rightVelocity4+';')
        socket.emit('velocity5', leftVelocity5+','+rightVelocity5+';')
        socket.emit('velocity6', leftVelocity6+','+rightVelocity6+';')
        socket.emit('velocity7', leftVelocity7+','+rightVelocity7+';')
        socket.emit('velocity8', leftVelocity8+','+rightVelocity8+';')
        socket.emit('velocity9', leftVelocity9+','+rightVelocity9+';')
        socket.emit('velocity10', leftVelocity10+','+rightVelocity10+';')
        socket.emit('velocity11', leftVelocity11+','+rightVelocity11+';')
        socket.emit('velocity12', leftVelocity12+','+rightVelocity12+';')
        socket.emit('velocity13', leftVelocity13+','+rightVelocity13+';')
        socket.emit('velocity14', leftVelocity14+','+rightVelocity14+';')
        socket.emit('velocity15', leftVelocity15+','+rightVelocity15+';')
        socket.emit('velocity16', leftVelocity16+','+rightVelocity16+';')
        socket.emit('velocity17', leftVelocity17+','+rightVelocity17+';')
        socket.emit('velocity18', leftVelocity18+','+rightVelocity18+';')
        socket.emit('velocity19', leftVelocity19+','+rightVelocity19+';')
        socket.emit('velocity20', leftVelocity20+','+rightVelocity20+';')
        count = count + 1;

        console.log('Robot1 send:'+rightVelocity1+","+leftVelocity1)
        console.log('Robot2 send:'+rightVelocity2+","+leftVelocity2)
        console.log('Robot3 send:'+rightVelocity3+","+leftVelocity3)
        console.log('Robot4 send:'+rightVelocity4+","+leftVelocity4)
        console.log('Robot5 send:'+rightVelocity5+","+leftVelocity5)
        console.log('Robot6 send:'+rightVelocity6+","+leftVelocity6)
        console.log('Robot7 send:'+rightVelocity7+","+leftVelocity7)
        console.log('Robot8 send:'+rightVelocity8+","+leftVelocity8)
        console.log('Robot9 send:'+rightVelocity9+","+leftVelocity9)
        console.log('Robot10 send:'+rightVelocity10+","+leftVelocity10)
        console.log('Robot11 send:'+rightVelocity11+","+leftVelocity11)
        console.log('Robot12 send:'+rightVelocity12+","+leftVelocity12)
        console.log('Robot13 send:'+rightVelocity13+","+leftVelocity13)
        console.log('Robot14 send:'+rightVelocity14+","+leftVelocity14)
        console.log('Robot15 send:'+rightVelocity15+","+leftVelocity15)
        console.log('Robot16 send:'+rightVelocity16+","+leftVelocity16)
        console.log('Robot17 send:'+rightVelocity17+","+leftVelocity17)
        console.log('Robot18 send:'+rightVelocity18+","+leftVelocity18)
        console.log('Robot19 send:'+rightVelocity19+","+leftVelocity19)
        console.log('Robot20 send:'+rightVelocity20+","+leftVelocity20)
        console.log(count)
          
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

  
