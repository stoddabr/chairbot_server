// legacy, ROS bridge deprecated 10/20 by Brett

// Connecting to ROS
// -----------------
// create a Ros node object to communicate with a rosbridge v2.0 server.

var ros_server = '10.214.152.222' // '192.168.1.196' // 'localhost' will only work if running on machine
var ros_port = '9090'
var ros = new ROSLIB.Ros({
  url: 'ws://' + ros_server + ':' + ros_port
});
console.log("Listening to " + ros_server + " on " + ros_port)

// add a listener for a connection event to the ros object.
ros.on('connection', function() {
  console.log('Connected to websocket server.');
});
ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});
ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});
// Publishing a Topic for each chair
// ------------------
var chair01 = new ROSLIB.Topic({
  ros: ros,
  name: '/cbon01',
  messageType: 'std_msgs/Int8'
});
var chair02 = new ROSLIB.Topic({
  ros: ros,
  name: '/cbon02',
  messageType: 'std_msgs/Int8'
});
var chair03 = new ROSLIB.Topic({
  ros: ros,
  name: '/cbon03',
  messageType: 'std_msgs/Int8'
});
var chair04 = new ROSLIB.Topic({
  ros: ros,
  name: '/cbon04',
  messageType: 'std_msgs/Int8'
});

var chairOn = new ROSLIB.Message({
  data: 1 //true
});
var chairOff = new ROSLIB.Message({
  data: 0 //false
});
// end legacy code

// define server requests
function enable(chairId) {
  $.post( "/toggle/enable/"+chairId );
}
function disable(chairId) {
  // disabled disable function so that chairs will continue to move
  // to their goals even when not selected
  // $.post( "/toggle/disable/"+chairId );
}

(function() {
  // Initialization: Set all chairs to be OFF. This means that everytime the webpage is loaded all the chairs ar OFF
  disable(1) // chair01.publish(chairOff);
  //document.getElementById("Switch01").checked = false;
  disable(2) // chair02.publish(chairOff);
  //document.getElementById("Switch02").checked = false;
  disable(3) // chair03.publish(chairOff);
  //document.getElementById("Switch03").checked = false;
  disable(4) // chair04.publish(chairOff);
  //document.getElementById("Switch04").checked = false;
})();

//Turn chair#1 ON by toggling if the switch is checked or not
function toggle_chair01() {
  var checkBox = document.getElementById("Switch01");
  if (checkBox.checked == true) {
    enable(1) // chair01.publish(chairOn);
    toggle_chair01_text();
    //toggle other checkboxes
    makeSelectAllIntuitive();

  } else {
    disable(1) // chair01.publish(chairOff);
    toggle_chair01_text();

    // if any of the chairs are unselected, uncheck select all.
    document.getElementById('SwitchAll').checked = false;
  }
}
//Give a textual feedback when chair#1 is ON/OFF by checking if the switch is checked or not
function toggle_chair01_text() {
  var checkBox = document.getElementById("Switch01");
  var text = document.getElementById("Toggle01");
  const chairText = 'Chair1' // update based on html, if not found default value
  if (checkBox.checked == true) {
    text.innerHTML = chairText + ",";
  } else {
    text.innerHTML = "";
  }
}


function toggle_chair02() {
  var checkBox = document.getElementById("Switch02");
  if (checkBox.checked == true) {
    enable(2) // chair02.publish(chairOn);
    toggle_chair02_text();
    //toggle other checkboxes
    makeSelectAllIntuitive();

  } else {
    disable(2) // chair02.publish(chairOff);
    toggle_chair02_text();
    // if any of the chairs are unselected, uncheck select all.
    document.getElementById('SwitchAll').checked = false;
  }
}

//Give a textual feedback when chair#2 is ON/OFF by checking if the switch is checked or not
function toggle_chair02_text() {
  var checkBox = document.getElementById("Switch02");
  var text = document.getElementById("Toggle02");
  const chairText = 'Chair2' // update based on html, if not found default value
  if (checkBox.checked == true) {
    text.innerHTML = chairText + ",";
  } else {
    text.innerHTML = "";
  }
}


function toggle_chair03() {
  var checkBox = document.getElementById("Switch03");
  if (checkBox.checked == true) {
    enable(3) // chair03.publish(chairOn);
    toggle_chair03_text();
    //toggle other checkboxes
    makeSelectAllIntuitive();

  } else {
    disable(3) // chair03.publish(chairOff);
    toggle_chair03_text();
    // if any of the chairs are unselected, uncheck select all.
    document.getElementById('SwitchAll').checked = false;
  }
}

//Give a textual feedback when chair#3 is ON/OFF by checking if the switch is checked or not
function toggle_chair03_text() {
  var checkBox = document.getElementById("Switch03");
  var text = document.getElementById("Toggle03");
  const chairText = 'Chair3' // update based on html, if not found default value
  if (checkBox.checked == true) {
    text.innerHTML = chairText + ",";
  } else {
    text.innerHTML = "";
  }
}


function toggle_chair04() {
  var checkBox = document.getElementById("Switch04");
  if (checkBox.checked == true) {
    enable(4) // chair04.publish(chairOn);
    toggle_chair04_text();
    //toggle other checkboxes
    makeSelectAllIntuitive();
  } else {
    disable(4) // chair04.publish(chairOff);
    toggle_chair04_text();
    // if any of the chairs are unselected, uncheck select all.
    document.getElementById('SwitchAll').checked = false;
  }
}

//Give a textual feedback when chair#4 is ON/OFF by checking if the switch is checked or not
function toggle_chair04_text() {
  var checkBox = document.getElementById("Switch04");
  var text = document.getElementById("Toggle04");
  const chairText = 'Chair4' // update based on html, if not found
  if (checkBox.checked == true) {
    text.innerHTML = chairText + ",";
  } else {
    text.innerHTML = "";
  }
}

function toggle_all() {
  var checkBox = document.getElementById("SwitchAll");
  if (checkBox.checked == true) { //check the checkbox
chairNamesArr = [1,2,3,4].map(el=>{ // get names from html
  const chairText = 'Chair'+el // update based on html, if not found default value
      return chairText + ", ";
});

    enable(1) // chair01.publish(chairOn);
    document.getElementById("Switch01").checked = true;
    document.getElementById("Toggle01").innerHTML = chairNamesArr[0]; // chairNamesArr is 0-indexed
    enable(2) // chair02.publish(chairOn);
    document.getElementById("Switch02").checked = true;
    document.getElementById("Toggle02").innerHTML =chairNamesArr[1]
    enable(3) // chair03.publish(chairOn);
    document.getElementById("Switch03").checked = true;
    document.getElementById("Toggle03").innerHTML = chairNamesArr[2];
    enable(4) // chair04.publish(chairOn);
    document.getElementById("Switch04").checked = true;
    document.getElementById("Toggle04").innerHTML = chairNamesArr[3];

  } else { //uncheck the checkbox
    disable(1) // chair01.publish(chairOff);
    document.getElementById("Switch01").checked = false;
    document.getElementById("Toggle01").innerHTML = "";
    disable(2) // chair02.publish(chairOff);
    document.getElementById("Switch02").checked = false;
    document.getElementById("Toggle02").innerHTML = "";
    disable(3) // chair03.publish(chairOff);
    document.getElementById("Switch03").checked = false;
    document.getElementById("Toggle03").innerHTML = "";
    disable(4) // chair04.publish(chairOff);
    document.getElementById("Switch04").checked = false;
    document.getElementById("Toggle04").innerHTML = "";
  }
}


// if all of the chairs are selected, selected all
function makeSelectAllIntuitive() {

  if (
    document.getElementById('Switch01').checked == true &&
    document.getElementById('Switch02').checked == true &&
    //document.getElementById('Switch03').checked == true &&
    document.getElementById('Switch04').checked == true
  ) {
    document.getElementById('SwitchAll').checked = true;
  }
}
