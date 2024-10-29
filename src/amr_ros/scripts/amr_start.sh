#!/bin/bash

#setup usb-rs485 adapter modbus cable
sudo -S su << EOF
mikuni
EOF
sudo su -c 'modprobe ftdi_sio vendor=0x06ce product=0x8331'
sudo su -c 'echo 06ce 8331 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'

sudo chmod 777 /dev/ttyUSB*


