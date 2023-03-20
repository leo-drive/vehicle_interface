#!/bin/bash

echo "Automatically bring up a SocketCAN interface on boot script started..."

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR

rm -f /etc/modules-load.d/can.conf
cp ./can.conf /etc/modules-load.d/can.conf

# Activate the systemd-networkd service
systemctl start systemd-networkd
systemctl enable systemd-networkd

# Configure the SocketCAN network interface with systemd-networkd
rm -f /etc/systemd/network/80-can.network
cp ./80-can.network /etc/systemd/network/80-can.network

sudo systemctl restart systemd-networkd

