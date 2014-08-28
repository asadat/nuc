if [ ! -e "stat" ]
then
	g++ stat.cpp -o stat
fi

for PATCH in `seq 1 1 4`
do
	for PERCENT in `seq 10 10 90` 
do
		cat $1 | grep $2 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*LENGTH //g" > tmp
		MEAN_VAR=$(./stat tmp)
		#echo "$1 PATCHES:$PATCH PERCENT:$PERCENT $MEAN_VAR"
	        echo "$PATCH $PERCENT $MEAN_VAR"
	done
done
