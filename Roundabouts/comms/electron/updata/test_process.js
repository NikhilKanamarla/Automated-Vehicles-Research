const { spawn }= require('child_process');				

child_process_list = spawn("python",["test_process.py"]);

child_process_list.stdout.on('data', function(data){
  console.log(`stdout: ${data}`);
});
child_process_list.stderr.on('data', function(data){
		console.log(`stderr: ${data}`);
});
child_process_list.on("close", function(){
	console.log("has been ended");
	
})
setTimeout(function() {
		child_process_list.kill()
		 }, 4000);
					  