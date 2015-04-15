#!/bin/bash

ip=192.168.0.142
port=40000
exe=$GAMS_ROOT/test_madara_reader
duration=10
file=1

$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
$exe -d $duration -u $ip:$port > $file.txt &
((port++))
((file++))
