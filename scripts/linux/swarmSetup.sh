#!/bin/sh
if [ $# != 3 ]
then
    echo "usage: $0 <ADAPTOR> <SSID> <ADDRESS>"
    echo "Sets the wireless adaptor to ad hoc mode with SSID and address"
else
    killall udhcpd
    ip link set $1 down; iwconfig $1 mode ad-hoc; iwconfig $1 essid $2; ifconfig $1 $3; ip link set $1 up
fi
