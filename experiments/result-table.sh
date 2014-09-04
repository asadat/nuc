if [ ! -e "stat" ]
then
	g++ stat.cpp -o stat
fi

for PATCH in `seq 1 1 4`
do
	for PERCENT in `seq 10 10 90` 
do
		cat $1 | grep $2 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.* LENGTH //g" | sed -e "s/ ASC.*//g" > tmp
		MEAN_VAR=$(./stat tmp)
		
		cat $1 | grep $2 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*Z_LENGTH: //g" | sed -e "s/ XY_LENGTH:.*//g" > tmp
		Z_LENGTH=$(./stat tmp)

		cat $1 | grep $2 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*XY_LENGTH: //g" > tmp
                XY_LENGTH=$(./stat tmp)

		echo "$PATCH $PERCENT $MEAN_VAR $Z_LENGTH $XY_LENGTH"
	done
done
