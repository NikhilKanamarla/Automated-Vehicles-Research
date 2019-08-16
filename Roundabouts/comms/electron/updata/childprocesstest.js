
const { spawn,exec }= require('child_process');
	
		child_process_list3 = spawn("bash",["rosvicon.sh"]);
		child_process_list4 = spawn("bash",["startcorners.sh"],{stdio: 'inherit' });
		child_process_list5 = spawn("bash",["startvectorfield.sh"],{stdio: 'inherit' });
		child_process_list6 = spawn("bash",["runmergecontrol.sh"],{stdio: 'inherit' });
		child_process_list7 = spawn("bash",["runmergesim.sh"],{stdio: 'inherit' });
		
		child_process_list3.on("close", function(){
			    console.log("has been ended");
		})
		child_process_list4.on("close", function(){
			    console.log("has been ended");
		})
		child_process_list5.on("close", function(){
			    console.log("has been ended");
		})
		child_process_list6.on("close", function(){
			    console.log("has been ended");
		})
		child_process_list7.on("close", function(){
			    console.log("has been ended");
		})



		setTimeout(function() {
    			child_process_list3.kill()
    			child_process_list4.kill()
    			child_process_list5.kill()
    			child_process_list6.kill()
    			child_process_list7.kill()
    			spawn("sh",["killprocess.sh"],{stdio: 'inherit' });
    			
   			 }, 10000);