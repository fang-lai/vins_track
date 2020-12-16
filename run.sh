#! /bin/bash

source ../../devel/setup.bash

# gnome-terminal	--window  -e 'bash -c "sleep 1s;rosrun vins vins_node ./config/helmet/helmet_mult.yaml ;exec bash"' \
gnome-terminal	--window  -e 'bash -c "sleep 1s;rosrun vins vins_node ./config/supernode/supernode.yaml ;exec bash"' \
		# --tab -e 'bash -c "sleep 1s;rosrun loop_fusion loop_fusion_node ./config/helmet/helmet_mult.yaml ;exec bash"'\
		# --tab -e 'bash -c "sleep 1s;rosbag play /home/jon/ubt_code/data/helmet/a.bag  ;exec bash"' \
		--tab -e 'bash -c "roslaunch vins vins_rviz.launch  ;exec bash"' 
		
		 
		

eval "$BASH_POST_RC"
