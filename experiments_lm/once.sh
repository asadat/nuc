#!/bin/bash

localros
roslaunch pelican_ctrl sim.launch &
sleep 5.0

echo > once.txt

for i in `seq 1 1 50`
do 
    echo "run $i"
    roslaunch nuc sim-batch.launch  >>  once.txt
done

killall roslaunch
killall roscore
killall rosmaster

