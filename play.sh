LOGFILE=$(echo $1 | sed "s/\(\/[_[:alnum:]]*\)/\1\1.log/g")
cat $LOGFILE | grep NEXT_WAY_POINT | sed "s/.*: //g" > wps.log
cat $LOGFILE | grep NEXT_WAY_POINT | sed "s/.*: //g"

cat $LOGFILE | grep POSE | sed "s/.*: //g" > poses.log

#logfolder=$(find "$1/*.jpg" | sed -e 's/\/[_[:alnum:]]*.log$//g')
PWDPATH=$(pwd)
#FILES=$(find "$1/*.jpg" | grep "[[:digit:]]\.jpg$")


#find $1/*.jpg

rm textures.log
touch textures.log
for f in `find $1/*.jpg | grep "[[:digit:]]-pred\.jpg$"`
do
	echo "$PWDPATH/$f" | sed -e "s/-pred//g" >> textures.log	
done

rm sensed.log
touch sensed.log
for f in `find $1/*.jpg | grep "[[:digit:]]-pred\.jpg$"`
do
	echo "$PWDPATH/$f" >> sensed.log
done

rosrun	nuc PlayLog wps.log poses.log textures.log sensed.log
#imageview &
#rosbag play -r 10 "$HOME/$1.bag" 
#webcamview &


