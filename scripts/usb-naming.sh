#!/bin/bash

echo "Port Align Script Started"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR

rm -f /etc/udev/rules.d/my-devices.rules
cp ./my-devices.rules /etc/udev/rules.d/my-devices.rules

udevadm control --reload-rules
udevadm trigger
/etc/init.d/udev restart

chmod 777 /dev/ttyLLC

cd ${HOME}/

