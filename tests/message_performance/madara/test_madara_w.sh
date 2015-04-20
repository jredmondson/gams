#!/bin/bash

duration=12
ip=192.168.0.142
port=40000
period=0.0001
exe=$GAMS_ROOT/test_madara_writer
cmd="$exe -d $duration -f $period -u 127.0.0.1:40560 "
for i in `seq 1 8`;
do
  cmd="$cmd -u $ip:$port"
  ((port++))
done
#cmd="$exe -d $duration -m 239.255.0.1:4150"

echo $cmd
$cmd
