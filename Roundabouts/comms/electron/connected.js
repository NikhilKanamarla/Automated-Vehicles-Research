// In renderer process (web page).
const {ipcRenderer} = require('electron')


ipcRenderer.on('asynchronous-reply', (event, arg1,arg2) => {
  document.getElementById('conntag'+arg2).innerHTML= arg1;
  document.getElementById('conntag'+arg2).setAttribute("class", arg1);
  if (arg1 == 'Disconnect') {
    document.getElementById('battery_icon'+arg2).src = "";
    document.getElementById("battery_lev"+arg2).innerHTML= '';
  } 
})
ipcRenderer.on('battery_level', (event, arg1, arg2) => {
	document.getElementById('battery_lev'+ arg2).innerHTML= '100%';
	if(arg1>'8000'){
		document.getElementById('battery_icon'+arg2).src = "image/battery84.png";
	}
	else if(arg1<'8000' && arg1 > '7500'){
		document.getElementById('battery_icon'+arg2).src = "image/battery82.png";
	}
	else if(arg1<'7500' && arg1 > '7000'){
		document.getElementById('battery_icon'+arg2).src = "image/battery80.png";
	}
	else if(arg1<'7000' && arg1 > '6250'){
		document.getElementById('battery_icon'+arg2).src = "image/battery83.png";
	}
	else{
		document.getElementById('battery_icon'+arg2).src = "image/battery81.png";
	}
   
 
})
ipcRenderer.send('asynchronous-message', 'ping', 'ping')


