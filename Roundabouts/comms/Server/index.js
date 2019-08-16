const {app,BrowserWindow,ipcMain} = require('electron');;
var io = require('socket.io')()
const path = require('path')
const url = require('url')
var rightVelocity='0';
var leftVelocity='0';


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
    var batt_lev = ''

  //to connect
    var ip = socket.request.connection.remoteAddress;
    var clientip = ip.substring(18);

//    var emitter = require('socket.io-emitter')("localhost:5001");
//    emitter.redis.on('error',function(err){
//        console.log(err);
//    });

    console.log("Socket" +"robot" +clientip + " connected !");
    win_sender.send('asynchronous-reply', 'connected', clientip)

  //sent velocities
 
    socket.on('getvelocity',function(velocities){
    rightVelocity = velocities.substring(0, velocities.indexOf(','));
    leftVelocity = velocities.substring(velocities.indexOf(',')+1, velocities.indexOf(';'));
    console.log(rightVelocity+","+leftVelocity)
    })


  //recive battery level from clients
    socket.on('battery', function(data){    
        batt_update_time = new Date().getTime();
        console.log('robot'+ clientip + " has battery level:" + data.toString());
        batt_lev = batt_lev + data.toString().substring(0,4)+'\t'
        win_sender.send('battery_level',data.toString(),clientip)
   });


  //disconnect handler
    var if_connected = setInterval(function(){ 
        var now = new Date().getTime();
        var disconnect_tester = now - batt_update_time;
        if(disconnect_tester > 8000){
          console.log("Socket " +"robot" +clientip + " disconnected.");
          clearInterval(if_connected);   
          win_sender.send('asynchronous-reply', 'Disconnect', clientip) 
          console.log('Robot'+clientip+":", batt_lev)
           
        }
        socket.emit('velocity', rightVelocity+','+leftVelocity+';')
        console.log(rightVelocity+","+leftVelocity)
    }, 20);
  
  });



ipcMain.on('asynchronous-message', (event, arg1, agr2) => {
   win_sender = event.sender;
})
  io.listen(5001);
  console.log("Server Started");
//setInterval(function(){ 
//    socket.emit('velocity', rightVelocity+','+leftVelocity+';')
//  },100)

  