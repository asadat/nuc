if [ ! -e "stat" ]
then
	g++ stat.cpp -o stat
fi

for PATCH in 1; do
	for PERCENT in 10 20 30 40 50 60 70 80 90; do
		cat results | grep $1 | grep "PATCHES:$PATCH" | grep "PERCENT:$PERCENT" | sed -e "s/.*LENGTH //g" > tmp
		MEAN_VAR=$(./stat tmp)
		#echo "$1 PATCHES:$PATCH PERCENT:$PERCENT $MEAN_VAR"
	        echo "$PATCH $PERCENT $MEAN_VAR"
	done
done
