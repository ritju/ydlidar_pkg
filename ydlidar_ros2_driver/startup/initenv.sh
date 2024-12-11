#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{devpath}=="6", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar_l"' >> /etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyUSB*", ATTRS{devpath}=="8", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar_r"' >> /etc/udev/rules.d/ydlidar.rules

# echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules

# echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules

service udev reload
sleep 2
service udev restart

