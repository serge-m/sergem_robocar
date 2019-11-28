const express = require('express');
const expressWS = require('express-ws');
const WebSocket = require('ws');
const path = require('path');
const fs = require('fs');
const app = express();
const appWS = expressWS(app);
const port = 3000;
const { spawn } = require('child_process');

const logCommandsOutput = true;
const numCharsToSave = 500;
const bagsDir = `${__dirname}/bags/`;

app.get('/', (request, response) => {
  console.log("root endpoint");
  response.send('Hello from Express!');;
})


app.get('/bags/', function(req, res) {
  const listBags = fs.readdirSync(bagsDir);
  res.send(listBags);
});

app.get('/bags/:name', function(req, res){
  const name = req.params.name;
  const listBags = fs.readdirSync(bagsDir);
  if (listBags.includes(name)) {
    const file = `${__dirname}/bags/${name}`;
    res.download(file);
  }
  else {
    res.send("Not found");
  }
});

app.use('/static', express.static(path.join(__dirname, 'static')))

function makeProcessInfo(command, args) {
  return {
    "command": command,
    "args": args,
    
    "process": null,
    "output": "",
    "update_function": null,
    "return_code": null
  };
}

var tracked_processes = {
  "dummy_script": makeProcessInfo('python', ['dummy_script.py']),
  "dummy_script_failing": makeProcessInfo('python', ['dummy_script_failing.py']),
  "rosbag record": makeProcessInfo(
    "sh", 
    ["-c", "rosbag record --split --duration=180s -o bags/robocar_recording_ /raspicam_node/image/compressed /pwm_radio_arduino/radio_pwm"]
  ),
  "roslaunch base": makeProcessInfo('roslaunch', ['/home/ubuntu/sergem_robocar/scripts/robocar_record.launch']),
  "AI driver": makeProcessInfo('rosrun', ['ai_driver_keras', 'ai_driver.py', '/home/ubuntu/sergem_robocar/linear_20191117.model']),
  "mode 0": makeProcessInfo('rostopic', ['pub', '/pwm_radio_arduino/mode',  'std_msgs/Int32', '--once', "data: 0"]),
  "mode 1": makeProcessInfo('rostopic', ['pub', '/pwm_radio_arduino/mode',  'std_msgs/Int32', '--once', "data: 1"]),
  "mode 2": makeProcessInfo('rostopic', ['pub', '/pwm_radio_arduino/mode',  'std_msgs/Int32', '--once', "data: 2"]),
  "mode 3": makeProcessInfo('rostopic', ['pub', '/pwm_radio_arduino/mode',  'std_msgs/Int32', '--once', "data: 3"])

};

var socket_clients = [];

function prepare_status_update() {
  all_updates = [];
  for (var name in tracked_processes) {
    var process_info = tracked_processes[name];
    var data_to_send = {
      "command": "update_status",
      "name": name,
      "status": "off",
      "data": process_info.output,
      "return_code": process_info.return_code
    };
    if (process_info.process) {
      data_to_send.status = "running";
    }
    all_updates.push(JSON.stringify(data_to_send));
  }
  return all_updates;
}

function update_clients() {
  all_updates = prepare_status_update();
  socket_clients.forEach((ws) => {
    if (ws.readyState !== WebSocket.OPEN) {
      return;
    }
    all_updates.forEach(
      (data_to_send) => { ws.send(data_to_send); }
    );   
  });
}

setInterval(update_clients, 1000);

function process_start(name) {
  var process_info = tracked_processes[name];
    if (process_info.process) {
      console.error("process " + name + " is already running");
      return;
    }
    console.error("starting process " + name);
    try {
      var p = spawn(process_info.command, process_info.args, { stdio: ['pipe'] });
    } catch (error) {
      process_info.output = `failed to launch. ${error}`;
      return;
    }
    
    function processOutput(data) {
      if (logCommandsOutput) {
        console.debug(name + ": " + data.toString().replace("\n", "\\n"));
      }
      process_info.output = (process_info.output + data.toString()).slice(-numCharsToSave);
    };
    p.stdout.on('data', processOutput);
    p.stderr.on('data', processOutput);

    p.on('close', (code) => {
      console.log(`process ${process_info.command} ${process_info.args} exited with code ${code}`);
      process_info.return_code = code;
      process_info.process = null;
    });
    p.on('error', (err) => {
      console.log(`process ${process_info.command} ${process_info.args} resulted in error ${err}`);
      process_info.return_code = null;
      process_info.output = `${err}`;
    });


    process_info.process = p;
    process_info.return_code = null;
}

function process_stop(name) {
  var process_info = tracked_processes[name];
    if (!process_info.process) {
      console.error("process " + name + " is not running");
      return;
    }
    console.error("stopping process " + name);
    process_info.process.kill();
    process_info.process = null;
}

function process_command(msg_json) {
  console.log("Processing ", msg_json);
  if (msg_json.command == "start") {
    return process_start(msg_json.name);
  }
  if (msg_json.command == "stop") {
    return process_stop(msg_json.name);
  }
}

app.ws('/', (ws, req) => {
  ws.on('message', function (msg) {
    try {
      msg_json = JSON.parse(msg);
      process_command(msg_json);
    } catch (error) {
      console.error(`Failed to process message ${msg}. Error ${error}. Continue processing...`);
    }
  });

  ws.on('error', function (err) {
    console.log('Found error: ' + err);
  });

  ws.on('close', function () {
    console.log('connection closed.');
  });

  socket_clients.push(ws);
  console.info("Client connected. num clients: ", socket_clients.length);
});


app.listen(port, (err) => {
  if (err) {
    return console.log('something bad happened', err)
  }
  console.log(`server is listening on ${port}`)
})


process.on('SIGINT', function () {
  console.log("terminating...")
  for (var name in tracked_processes) {
    process_stop(name);
  }
  update_clients();
  process.exit();
});