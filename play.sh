cat "$1.log" | grep WAY | sed "s/.*: //" > wps.log
cat "$1.log" | grep WAY | sed "s/.*: //"
rosrun	nuc PlayLog wps.log &
imageview &
rosbag play -r 10 "$HOME/$1.bag" 
#webcamview &


