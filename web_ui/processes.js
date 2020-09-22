const { spawn } = require("child_process");

const logCommandsOutput = true;
const numCharsToSave = 500;

function makeProcessInfo(name, command, args) {
  return {
    name: name,
    command: command,
    args: args,

    process: null,
    output: "",
    update_function: null,
    return_code: null,
  };
}

var tracked_processes = Object.fromEntries(
  [
    makeProcessInfo("rosbag record", "sh", [
      "-c",
      "rosbag record --split --duration=60s -o bags/robocar_recording_ /raspicam_node/image/compressed /pwm_radio_arduino/control_result",
    ]),
    makeProcessInfo("roslaunch base", "roslaunch", [
      "../scripts/robocar_record.launch",
    ]),
    makeProcessInfo("AI driver", "roslaunch", [
      "../catkin_ws/src/ai_pilot_keras/launch/ai_pilot_keras.launch",
    ]),
    makeProcessInfo("mode 0", "rostopic", [
      "pub",
      "/pwm_radio_arduino/mode",
      "std_msgs/Int32",
      "--once",
      "data: 0",
    ]),
    makeProcessInfo("mode 1", "rostopic", [
      "pub",
      "/pwm_radio_arduino/mode",
      "std_msgs/Int32",
      "--once",
      "data: 1",
    ]),
    makeProcessInfo("mode 2", "rostopic", [
      "pub",
      "/pwm_radio_arduino/mode",
      "std_msgs/Int32",
      "--once",
      "data: 2",
    ]),
    makeProcessInfo("mode 3", "rostopic", [
      "pub",
      "/pwm_radio_arduino/mode",
      "std_msgs/Int32",
      "--once",
      "data: 3",
    ]),

    makeProcessInfo("dynamic_reconfigure", "rosrun", [
      "dynamic_reconfigure",
      "dynparam",
      "set",
      "/steering_translator_node",
      "{params}",
    ]),

    makeProcessInfo("shutdown", "sudo", ["poweroff"]),
    makeProcessInfo("dummy_script", "python", ["dummy_script.py"]),
    makeProcessInfo("dummy_script_failing", "python", [
      "dummy_script_failing.py",
    ]),
  ].map((pi) => [pi.name, pi])
);

function replace_templates(strings, params) {
  return strings.map((str) => str.replace("{params}", params));
}

function start(name, params) {
  var process_info = tracked_processes[name];
  if (!process_info) {
    console.error(`process with name ${name} was not found`);
    return;
  }
  if (process_info.process) {
    console.error("process " + name + " is already running");
    return;
  }
  const args = replace_templates(process_info.args, params);
  console.error(
    `starting process ${name}: command ${process_info.command} args ${args}`
  );
  try {
    var p = spawn(process_info.command, args, { stdio: ["pipe"] });
  } catch (error) {
    process_info.output = `failed to launch. ${error}`;
    return;
  }

  function processOutput(data) {
    if (logCommandsOutput) {
      console.debug(name + ": " + data.toString().replace("\n", "\\n"));
    }
    process_info.output = (process_info.output + data.toString()).slice(
      -numCharsToSave
    );
  }

  p.stdout.on("data", processOutput);
  p.stderr.on("data", processOutput);

  p.on("close", (code) => {
    console.log(`process ${process_info.name} exited with code ${code}`);
    process_info.return_code = code;
    process_info.process = null;
  });
  p.on("error", (err) => {
    console.log(`process ${process_info.name} resulted in error ${err}`);
    process_info.return_code = null;
    process_info.output = `${err}`;
  });

  process_info.process = p;
  process_info.return_code = null;
}

function stop(name) {
  var process_info = tracked_processes[name];
  if (!process_info.process) {
    console.error("process " + name + " is not running");
    return;
  }
  console.error("stopping process " + name);
  process_info.process.kill();
  process_info.process = null;
}

module.exports = {
  tracked_processes,
  start,
  stop,
};
