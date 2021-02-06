/**
 * note: ROSLIB is no longer being used
 * all calls to it need to be sergically removed - Brett 12/11
 */

/*
var requestMotion01 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestMotion01',
	messageType : 'std_msgs/String'
});
var requestMotion02 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestMotion02',
	messageType : 'std_msgs/String'
});
var requestMotion03 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestMotion03',
	messageType : 'std_msgs/String'
});
var requestMotion04 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestMotion04',
	messageType : 'std_msgs/String'
});

var requestStop01 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestStop01',
	messageType : 'std_msgs/String'
});
var requestStop02 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestStop02',
	messageType : 'std_msgs/String'
});
var requestStop03 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestStop03',
	messageType : 'std_msgs/String'
});
var requestStop04 = new ROSLIB.Topic({
	ros : ros,
	name: '/requestStop04',
	messageType : 'std_msgs/String'
});
*/

/* These are the constants recognized by our python ros api including the packet replicator */
const FORWARD = 'FORWARD'
const BACKWARD = 'BACKWARD'
const RIGHT = 'RIGHT'
const LEFT=  'LEFT'
const STOP = 'STOP'

function sendMotion(motion, chairId) {
	const motionstr = String(motion)
	const chairIdstr = String(chairId)
  $.post( "/move/"+motionstr+'/'+chairIdstr);
}

function askToRunSequence(motion) {
	//console.log("The requested motion is:" + motion);
	//what chairs are we sending the data to ?
	//figure out based on if they are checked or not
	chairs = $('.chair_selectors')

	if (motion == STOP) { //this is the STOP command
		//encode the packet to make it processesable by ROS
		// requestStopPacket = new ROSLIB.Message({data: motion})
		//console.log("Stop Packet is ", requestStopPacket)

		//send it to those chair's topic which were selected
		for (var chair of chairs)
		{
			if (chair.checked == true)
			{
				console.log("Sending STOP to " + chair.id)
				sendMotion(motion, chair.id.substr(-1))
				/* legacy code from using ROSjslib socket
				// console.log("Sending STOP to " + chair.id)
				//construct the topic name by using hte chair id
				requestStop_topic_name = 'requestStop' + chair.id.substr(-2)
				//actually get the topic object for that chair's requestStopTopic and then publish
				eval(requestStop_topic_name).publish(requestStopPacket);
				//this is for testing
				eval(requestStop_topic_name).subscribe(printData)
				*/
			}
			else
			{
				// console.log("NOT Sending command to " + chair.id)
			}
		}
	}
	else
	{
		//encode the packet to make it processesable by ROS
		// requestMotionPacket = new ROSLIB.Message({data: motion})
		// console.log("Motion Packet is " + requestMotionPacket)

		//send it to those chair's topic which were selected
		for (var chair of chairs)
		{
			if (chair.checked == true)
			{
				//construct the topic name by using hte chair id
				// requestMotion_topic_name = 'requestMotion' + chair.id.substr(-2)
				//actually get the topic object for that chair's requestMotionTopic and then publish
				//console.log("Sending MOTION command to ", chair.id, ' through ', requestMotion_topic_name)
				console.log('Sending MOTION command',{motion, id:chair.id});
				sendMotion(motion, chair.id.substr(-1))
				// eval(requestMotion_topic_name).publish(requestMotionPacket);

				//this is for testing
				// eval(requestMotion_topic_name).subscribe(printData)
			}
			else
			{
				// console.log("NOT Sending command to " + chair.id)
			}
		}

	}
}

function printData(message) {
	//TODO: Send this to the ROS logging facility
	console.log("Received msg which said " + message.data);
}


function forward() {
	askToRunSequence(FORWARD);
}
function backward() {
	askToRunSequence(BACKWARD);
}
function stop() {
	askToRunSequence(STOP);
}
function turnRight() {
	askToRunSequence(RIGHT);
}
function turnLeft() {
	askToRunSequence(LEFT);
}
