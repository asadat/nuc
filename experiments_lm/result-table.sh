if [ ! -e "stat" ]
then
	g++ stat.cpp -o stat
fi

for PATCH in `seq 1 2 20`
do
	for PERCENT in `seq 20 20 60` 
	do
		
		cat $1 | grep $2 | grep "LML: $3" | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.* LENGTH //g" | sed -e "s/ ASC.*//g" > tmp
		MEAN_VAR=$(./stat tmp)
		
		cat $1 | grep $2 | grep "LML: $3" | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*Z_LENGTH: //g" | sed -e "s/ XY_LENGTH:.*//g" > tmp
		Z_LENGTH=$(./stat tmp)

		cat $1 | grep $2 | grep "LML: $3" | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*XY_LENGTH: //g" | sed -e "s/ TURNS:.*//g" > tmp
                XY_LENGTH=$(./stat tmp)

		cat $1 | grep $2 | grep "LML: $3" | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*TURNS: //g" | sed -e "s/ LML:.*//g" > tmp
                TURNS=$(./stat tmp)

		echo "$PATCH $PERCENT $MEAN_VAR $Z_LENGTH $XY_LENGTH $TURNS"
	done
done
