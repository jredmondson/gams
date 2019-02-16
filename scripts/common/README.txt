This folder contains default parameters and configuration options for
MADARA tools to use whenever you load them. The parameter files require
each argument pairing to be on a separate line.

For instance, if you want to configure the karl command line tool to always
include printing resulting knowledge, waiting for 30 seconds, and also
specifying a multicast ip that is commonly used for GAMS tools, you would
create the following file:

in $HOME/.madara/karl
-k
-m 239.255.0.1:4150
-w 30

stk_inspect parameters can be added in the $HOME/.madara/stk_inspect file.

For more information on the karl tool, please see the following:

1) karl built-in help (-h):         $MADARA_ROOT/bin/karl -h
2) stk_inspect built-in help (-h):  $MADARA_ROOT/bin/stk_inspect -h
3) Debugging with karl wiki: https://github.com/jredmondson/madara/wiki/Debugging-With-Karl
4) STK Inspect wiki: https://github.com/jredmondson/madara/wiki/STK-Inspect

REMEMBER: if you set something in the config file, it is loaded every time you
run the respective tool. So, if you do something like -m 239.255.0.1:4150 in
the config file, and then you decide you need to do UDP transports one day,
you'll need to clear the entry in the config file or it will add a
multicast IP:port to your host list and things won't work the way you're
expecting them to (you'll basically have host[0]=MULTICASTIP:PORT,
host[1]=UDPIP:PORT, host[2]=UDPIP:PORT instead of host[0]=UDPIP:PORT,
host[1]=UDPIP:PORT). If you delete -m 239.255.0.1:4150 from the config 
file, then everything will work properly.