# robocar

Self-driving toy car based on ROS and RaspberryPi.

Robocar is an attempt to replicate [donkeycar](https://www.donkeycar.com/) project with [ROS](https://www.ros.org/) using similar hardware.

[project-omicron](https://github.com/project-omicron/robocar) was used as a base. See `thirdparty` directory for the thirdparty and licence information.



# Getting started

Get the code
```
git clone https://github.com/serge-m/sergem_robocar.git robocar
cd robocar
git submodule update  --init
```

Follow [installation instructions in wiki](https://github.com/serge-m/sergem_robocar/wiki).



# Architecture

![serge-m robocar architecture](/doc/img/sergem-robocar-diagram.svg)

The Robocar runs on [Robot Operating System](https://www.ros.org/) (ROS).
I use Ubuntu image with [preinstalled ROS from ubiquity](https://downloads.ubiquityrobotics.com/pi.html).

The system consists of the following nodes.
* `raspicam_node` - captures the images from Raspberry Pi camera and publishes to a ROS topic
* `ai_driver` - takes the image and produces steering and throttle values. The values are normalized. The range is (-1, 1).
* `steering_translator` - an auxiliary node that converts normalized steering to PWM signal for the servos.
* `arduino bridge` - listens to the converted steering topic and publishes the data to the arduino via serial port (USB). In addition it serves as a bridge for the PWM signals captured by arduino from RC-receiver.
* Arduino sketch takes care of several things:
    * measuring PWM signal from RC-receiver,
    * publishing this received PWM to ROS
    * listening for PWM commands produced by `ai_driver` and translated by `steering_translator`
    * listening for a special topic that controls which PWM to use (one from radio or one from `ai_driver`)
    * shuts down throttle if there is no RC signal (safety measure)

For the further info see [wiki](https://github.com/serge-m/sergem_robocar/wiki).
