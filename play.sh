cat "$1.log" | grep WAY | sed "s/.*: //g" > wps.log
cat "$1.log" | grep WAY | sed "s/.*: //g"

cat "$1.log" | grep POSE | sed "s/.*: //g" > poses.log

rosrun	nuc PlayLog wps.log poses.log
#imageview &
#rosbag play -r 10 "$HOME/$1.bag" 
#webcamview &


