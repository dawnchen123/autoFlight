#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# su
# echo "A" | sudo -S netstat -tlnp
# modprobe uvcvideo
# chmod 777 /dev/ttyTHS2
# chmod 777 /dev/rplidar

cd /home/dji/autoFlight/src/autoControl/aruco-3.1.3/build/utils
./aruco_ros live[1] camera.yml -s 0.1 -d ARUCO_MIP_36h12
exit 0
