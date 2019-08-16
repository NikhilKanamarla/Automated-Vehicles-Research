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


// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.


function createWindow () {
  // Create the browser window.
  mainWindow = new BrowserWindow({width: 1000, height: 600})

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
      //console.log('Robot1:'+rightVelocity1+","+leftVelocity1+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '2:'){
      rightVelocity2 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity2 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot2:'+rightVelocity2+","+leftVelocity2+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '3:'){
      rightVelocity3 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity3 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot3:'+rightVelocity3+","+leftVelocity3+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '4:'){
      rightVelocity4 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity4 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot4:'+rightVelocity4+","+leftVelocity4+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '5:'){
      rightVelocity5 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity5 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot5:'+rightVelocity5+","+leftVelocity5+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '6:'){
      rightVelocity6 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity6 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot6:'+rightVelocity6+","+leftVelocity6+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '7:'){
      rightVelocity7 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity7 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot7:'+rightVelocity7+","+leftVelocity7+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '8:'){
      rightVelocity8 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity8 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot8:'+rightVelocity8+","+leftVelocity8+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '9:'){
      rightVelocity9 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity9 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot9:'+rightVelocity9+","+leftVelocity9+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '10'){
      rightVelocity10 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity10 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot10:'+rightVelocity10+","+leftVelocity10+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '11'){
      rightVelocity11 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity11 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot11:'+rightVelocity11+","+leftVelocity11+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '12'){
      rightVelocity12 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity12 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot12:'+rightVelocity12+","+leftVelocity12+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '13'){
      rightVelocity13 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity13 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot10:'+rightVelocity13+","+leftVelocity13+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '14'){
      rightVelocity14 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity14 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot14:'+rightVelocity14+","+leftVelocity14+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '15'){
      rightVelocity15 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity15 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot15:'+rightVelocity15+","+leftVelocity15+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '16'){
      rightVelocity16 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity16 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot16:'+rightVelocity16+","+leftVelocity16+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '17'){
      rightVelocity17 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity17 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot17:'+rightVelocity17+","+leftVelocity17+'recived!!!!!!!!!')
    }   
    if (velocities.substr(0,2) == '18'){
      rightVelocity18 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity18 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot18:'+rightVelocity18+","+leftVelocity18+'recived!!!!!!!!!')
    }    
    if (velocities.substr(0,2) == '19'){
      rightVelocity19 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity19 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot19:'+rightVelocity19+","+leftVelocity19+'recived!!!!!!!!!')
    }    
    if (velocities.substr(0,2) == '20'){
      rightVelocity20 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity20 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }
    if (velocities.substr(0,2) == '21'){
      rightVelocity21 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity21 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '22'){
      rightVelocity22 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity22 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '23'){
      rightVelocity23 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity23 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '24'){
      rightVelocity24 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity24 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '25'){
      rightVelocity25 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity25 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '26'){
      rightVelocity26 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity26 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    } 
    if (velocities.substr(0,2) == '27'){
      rightVelocity27 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity27 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '28'){
      rightVelocity28 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity28 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '29'){
      rightVelocity29 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity29 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }     
    if (velocities.substr(0,2) == '30'){
      rightVelocity30 = velocities.substring(velocities.indexOf(':')+1, velocities.indexOf(','));
      leftVelocity30 = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
      //console.log('Robot20:'+rightVelocity20+","+leftVelocity20+'recived!!!!!!!!!')
    }                                                      
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
        socket.emit('velocity21', leftVelocity21+','+rightVelocity21+';')
        socket.emit('velocity22', leftVelocity22+','+rightVelocity22+';')
        socket.emit('velocity23', leftVelocity23+','+rightVelocity23+';')
        socket.emit('velocity24', leftVelocity24+','+rightVelocity24+';')
        socket.emit('velocity25', leftVelocity25+','+rightVelocity25+';')
        socket.emit('velocity26', leftVelocity26+','+rightVelocity26+';')
        socket.emit('velocity27', leftVelocity27+','+rightVelocity27+';')
        socket.emit('velocity28', leftVelocity28+','+rightVelocity28+';')
        socket.emit('velocity29', leftVelocity29+','+rightVelocity29+';')
        socket.emit('velocity30', leftVelocity30+','+rightVelocity30+';')
        count = count + 1;
        socket.emit('batt', batt1)
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
        //console.log('Robot20 send:'+rightVelocity20+","+leftVelocity20)
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

  
