#if [ ! -e "stat" ]
#then
#	g++ stat.cpp -o stat
#fi

for PATCH in `seq 1 1 4`
do
	for PERCENT in `seq 10 10 90` 
	do
		cat $1 | grep $2 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.* FNP: //g" > tmp
		MEAN_VAR=$(./stat tmp)
		
		echo "$PATCH $PERCENT $MEAN_VAR"
	done
done
