#!/usr/bin/env bash
mkdir serial_hw_interface_ws 
cd serial_hw_interface_ws 
mkdir src   
cd src 
git clone https://github.com/everyrobot/serial.git  
git clone https://github.com/everyrobot/er_ti_f28069m_drv8305.git
git clone https://github.com/everyrobot/serial-example.git
cd serial-example
git fetch
git checkout -b ros
git pull origin ros
cd ..
git clone https://github.com/everyrobot/tr1_essentials.git
cd tr1_essentials/
git fetch
git checkout -b ros
git pull origin ros
cd ../.. 
catkin_make
source devel/setup.bash
gnome-terminal -x sh -c "roscore"
sleep 1.5
roslaunch tr1_hardware_interface tr1_effort_controllers.launch
