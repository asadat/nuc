#!/bin/bash

#export ROS_MASTER_URI=http://localhost:22322
#roscore -p 22322 &
#sleep 5.0
localros
roslaunch pelican_ctrl sim.launch &
sleep 5.0

for CONF in `seq 6 1 7`
do

echo > "results-$CONF"

for PATCHES in `seq 1 1 4` 
do
	for STRATEGY in `seq 3 1 3`  
	do
		for PERCENT in `seq 10 10 90` 
		do		

			for i in `seq 1 1 10`
			do 
			    cat param.yaml > ../launch/batchrun_param.yaml
			    echo "patches: $PATCHES" >> ../launch/batchrun_param.yaml
			    echo "percent_interesting: $PERCENT" >> ../launch/batchrun_param.yaml
			    
			    echo "conf1: $CONF"		
			    if [ $CONF -eq 1 ]; then
				echo "alpha_h0: 0.001" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.01" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.001" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.01" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 2 ]; then
				echo "alpha_h0: 0.1" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.4" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.1" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.4" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 3 ]; then
				echo "alpha_h0: 0.1" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.4" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.001" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.01" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 4 ]; then
				echo "alpha_h0: 0.001" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.01" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.1" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.4" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 5 ]; then
				echo "alpha_h0: 0.2" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.3" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.2" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.3" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 6 ]; then
				echo "alpha_h0: 0.15" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.4" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.15" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.4" >> ../launch/batchrun_param.yaml
			    fi

			    if [ $CONF -eq 7 ]; then
				echo "alpha_h0: 0.2" >> ../launch/batchrun_param.yaml
				echo "alpha_hm: 0.4" >> ../launch/batchrun_param.yaml
				echo "beta_h0: 0.2" >> ../launch/batchrun_param.yaml
				echo "beta_hm: 0.4" >> ../launch/batchrun_param.yaml
			    fi

			    echo "strategy: hi"
                    	    echo "strategy: hi" >> ../launch/batchrun_param.yaml

                 

			    echo "run:$i strategy:$STRATEGY patches:$PATCHES percent:$PERCENT CONF:$CONF" 
			    roslaunch nuc batchrun.launch  | grep "STRATEGY" >> "results-$CONF"
 			    
			done
		done
	done
done
done

killall roslaunch
killall roscore
killall rosmaster

