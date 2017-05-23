#!/bin/bash

DRV_NAME=fbsrc

# Rebuild driver and install it
make
sudo mkdir -p /lib/modules/$(uname -r)/extra
sudo cp ./$DRV_NAME.ko /lib/modules/$(uname -r)/extra
sudo depmod -a
sync

# Unload driver if necesary
lsmod | grep $DRV_NAME > /dev/null && sudo rmmod $DRV_NAME

# Load driver
sudo modprobe $DRV_NAME
