# robocar
Main repository for the robocar

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



### Possible problems
* ImportError: libhdf5_serial.so.100: cannot open shared object file: No such file or directory
  Try to install h5py with a fixed version of HDF5:
  ```
  pip uninstall h5py
  HDF5_VERSION=1.8.16 pip install --no-binary=h5py h5py
  ```