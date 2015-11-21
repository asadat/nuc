#!/bin/bash

localros
roslaunch pelican_ctrl sim.launch &
sleep 5.0
cat ../launch/batchrun_param.yaml
echo > results
#for PATCHES in `seq 1 1 1` 
#do
	for STRATEGY in `seq 3 1 3`  
	do
		#for PERCENT in `seq 10 20 90` 
		#do		

			#for i in `seq 1 2 10`
			#do 
			    cat param.yaml > ../launch/batchrun_param.yaml
			    #echo "patches: $PATCHES" >> ../launch/batchrun_param.yaml
			    #echo "percent_interesting: $PERCENT" >> ../launch/batchrun_param.yaml
		
			    if [ $STRATEGY -eq 1 ]; then
				    echo "policy: greedy"
				    echo "policy: greedy" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $STRATEGY -eq 2 ]; then
				    echo "policy: delayed_greedy"
                    		    echo "policy: delayed_greedy" >> ../launch/batchrun_param.yaml
                	    fi

			    if [ $STRATEGY -eq 3 ]; then
				    echo "policy: delayed"
                    		    echo "policy: delayed" >> ../launch/batchrun_param.yaml
                 	    fi
                 
			    #echo "run:$i strategy:$STRATEGY patches:$PATCHES percent:$PERCENT" 
			    roslaunch nuc batchrun.launch  #| grep "resolution" 
 			    
			#done
		#done
	#done
done

killall roslaunch
killall roscore
killall rosmaster

