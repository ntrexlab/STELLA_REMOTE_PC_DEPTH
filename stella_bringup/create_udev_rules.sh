#!/bin/bash

echo "remap the devices serial port(ttyUSBX, ttySX, videoX) to  ydlidar, Astra Pro, AHRS, Motordriver, Bluetooth, camera"
echo "devices usb connection as /dev/YDLIDAR, /dev/AHRS, /dev/MW, /dev/BT, /dev/astra_pro, /dev/CAMERA, check it using the command : ls -l /dev|grep -e ttyUSB -e ttyS -e video"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "`rospack find stella_bringup`/stella.rules"
sudo cp `rospack find stella_bringup`/stella.rules  /etc/udev/rules.d
echo "start copy camera_info to  ~/.ros"
echo "`rospack find stella_bringup`/camera_info"
sudo cp -r `rospack find stella_bringup`/camera_info ~/.ros
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
