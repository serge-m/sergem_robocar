const express = require('express');
const expressWS = require('express-ws');
const WebSocket = require('ws');
const path = require('path');
const fs = require('fs');
const app = express();
const appWS = expressWS(app);
const port = 3000;

const processes = require('./processes.js');


const bagsDir = `${__dirname}/bags/`;

app.get('/', (request, response) => {
  console.log("root endpoint");
  response.send('Hello from Express!');;
})


app.get('/bags/', function (req, res) {
  const listBags = fs.readdirSync(bagsDir);
  res.send(listBags);
});

app.get('/bags/:name', function (req, res) {
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



var socket_clients = [];

function commandUpdate(process_info) {
  var command = {
    "command": "update_status",
    "name": process_info.name,
    "status": "off",
    "data": process_info.output,
    "return_code": process_info.return_code
  };
  if (process_info.process) {
    command.status = "running";
  }
  return command;
}

function commandListCommands() {
  return {
    "command": "update_command_list",
    "names": Array.from(Object.keys(processes.tracked_processes))
  };
}

function prepare_status_update() {
  return Object.values(processes.tracked_processes).map(process_info => 
    JSON.stringify(commandUpdate(process_info))
    );
}

function update_clients() {
  all_updates = prepare_status_update();
  socket_clients.forEach((ws) => {
    if (ws.readyState !== WebSocket.OPEN) {
      return;
    }
    all_updates.forEach(data => ws.send(data));
  });
}

setInterval(update_clients, 1000);



function process_command(msg_json) {
  console.log("Processing ", msg_json);
  if (msg_json.command == "start") {
    return processes.start(msg_json.name, msg_json.params);
  }
  if (msg_json.command == "stop") {
    return processes.stop(msg_json.name);
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
  const commandListCmd = JSON.stringify(commandListCommands());
  console.info(`Sendind list of commands: ${commandListCmd}`);
  ws.send(commandListCmd);
});


app.listen(port, (err) => {
  if (err) {
    return console.log('something bad happened', err)
  }
  console.log(`server is listening on ${port}`)
})


process.on('SIGINT', function () {
  console.log("terminating...")
  for (var name in processes.tracked_processes) {
    processes.stop(name);
  }
  update_clients();
  process.exit();
});