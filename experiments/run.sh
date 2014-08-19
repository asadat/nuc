#!/bin/bash

localros
roslaunch pelican_ctrl sim.launch &
sleep 5.0

echo > results
for PATCHES in 1  
do
	for STRATEGY in 1 2 3 
	do
		for PERCENT in 10 20 30 40
		do 
			cat param.yaml > ../launch/batchrun_param.yaml
			echo "patches: $PATCHES" >> ../launch/batchrun_param.yaml
			echo "percent_interesting: $PERCENT" >> ../launch/batchrun_param.yaml
		
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
			roslaunch nuc batchrun.launch  | grep "STRATEGY" >> results
		done
	done
done

killall roslaunch
