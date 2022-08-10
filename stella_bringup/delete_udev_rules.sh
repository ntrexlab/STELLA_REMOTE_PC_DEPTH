#!/bin/bash

echo "delete remap the devices serial port(ttyUSBX,ttySX, videoX) to  ydlidar, Astra Pro, AHRS, Motordriver, Bluetooth, camera"
echo "sudo rm   /etc/udev/rules.d/stella.rules"
sudo rm   /etc/udev/rules.d/stella.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish  delete"
