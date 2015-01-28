#!/bin/bash

#export ROS_MASTER_URI=http://localhost:22322
#roscore -p 22322 &
#sleep 5.0
localros
roslaunch pelican_ctrl sim.launch &
sleep 5.0

echo > results
for PATCHES in `seq 1 2 20` 
do
	for STRATEGY in `seq 3 1 3`  
	do
		for PERCENT in `seq 20 10 60` 
		do
			for LML in `seq 0 1 4`
			do		
				for i in `seq 1 1 10`
				do 
				    cat param.yaml > ../launch/batchrun_param.yaml
				    echo "patches: $PATCHES" >> ../launch/batchrun_param.yaml
				    echo "percent_interesting: $PERCENT" >> ../launch/batchrun_param.yaml
				    echo "LML_start: $LML" >> ../launch/batchrun_param.yaml

				    if [ $STRATEGY -eq 1 ]; then
					    echo "strategy: df"
					    echo "strategy: df" >> ../launch/batchrun_param.yaml
				    fi

				    if [ $STRATEGY -eq 2 ]; then
					    echo "strategy: sc"
		            		    echo "strategy: sc" >> ../launch/batchrun_param.yaml
		        	    fi

				    if [ $STRATEGY -eq 3 ]; then
					    echo "strategy: hi"
		            		    echo "strategy: hi" >> ../launch/batchrun_param.yaml
		         	    fi
		         
				    echo "run:$i strategy:$STRATEGY patches:$PATCHES percent:$PERCENT LML:$LML" 
				    roslaunch nuc batchrun.launch  | grep "STRATEGY" >> results
	 			    
				done
			done
		done
	done
done

killall roslaunch
killall roscore
killall rosmaster

