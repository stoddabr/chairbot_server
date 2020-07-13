var label01 = new ROSLIB.Topic({
	ros : ros,
	name: '/label01',
	messageType : 'std_msgs/String'
});
var label02 = new ROSLIB.Topic({
	ros : ros,
	name: '/label02',
	messageType : 'std_msgs/String'
});
var label03 = new ROSLIB.Topic({
	ros : ros,
	name: '/label03',
	messageType : 'std_msgs/String'
});

function labelPositiveInteraction(){
	console.log("Positive interaction")
}
function labelInterestingInteraction(){
	console.log("Interesting interaction")
}
function labelNegativeInteraction(){
	console.log("Negative interaction")
}