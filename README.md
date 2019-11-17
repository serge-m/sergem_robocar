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
This is a modified version of the robocar.
![serge-m robocar architecture](/doc/img/sergem-robocar-diagram.svg)

For further docs see the wiki.

## Installation on Raspberry Pi with Ubiquity image
* Download and deploy the ubiquity image on your arspberry
* sudo apt install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libjpeg62

* sudo apt install python3-venv
cd ~/robocar/
 python3 -m venv py3
source py3/bin/activate
 pip install --upgrade pip

pip install rospkg tensorflow pillow
mkdir -p $HOME/.config/pip/
echo -e "[global]\nextra-index-url=https://www.piwheels.org/simple\n" >> $HOME/.config/pip/pip.conf

pip install rospkg tensorflow


* sudo apt install ros-kinetic-ackermann-msgs


cd ~/robocar/catkin_ws/
catkin_make build
source devel/setup.bash



roslaunch ./scripts/robocar.launch


For the each restart of roscore load steering-PWM translation parameters:
rosrun dynamic_reconfigure dynparam load /steering_translator_node catkin_ws/src/steering_translator/steering_translator_sample.yaml


By default the car is controlled by the radio. 

To enable AI driver run
rostopic pub /pwm_radio_arduino/mode std_msgs/Int32 --once "data: 2"

There is a safer mode that stops the car as soon as the radio receiver is off:
rostopic pub /pwm_radio_arduino/mode std_msgs/Int32 --once "data: 3"


## Camera setup

Preview images from raspicam:
```
rqt_image_view
```

Set up camera parammeters with 
```
rosrun rqt_reconfigure rqt_reconfigure
```

Save parameters file 

## Recording data

Run required nodes with
```
roslaunch ./scripts/robocar_record.launch
```

Load camera parameters saved in the "Camera setup" section if needed:
```
rosrun dynamic_reconfigure dynparam load /raspicam_node camera_settings.yaml
```


Start recording:
```
rosbag record --split --duration=180s -o robocar_recording_ /raspicam_node/image/compressed /pwm_radio_arduino/radio_pwm
```

## UI for parameters setup
rosrun rqt_reconfigure rqt_reconfigure


## Checking data values
```
rqt_plot
```

or (for /pwm_radio_arduino/driver_pwm)

```
rostopic echo /pwm_radio_arduino/driver_pwm
```
 

### Possible problems
* ImportError: libhdf5_serial.so.100: cannot open shared object file: No such file or directory
  Try to install h5py with a fixed version of HDF5:
  ```
  pip uninstall h5py
  HDF5_VERSION=1.8.16 pip install --no-binary=h5py h5py
  ```
