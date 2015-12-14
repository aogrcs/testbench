#!/bin/bash

cd ~/catkin_ws
(roscore & echo $! >> /tmp/mainpids) &
(sleep 5; roslaunch bert2_simulator bert2_gazebo.launch & echo $! >> /tmp/mainpids) &
(sleep 10; roslaunch bert2_moveit move_group.launch & echo $! >> /tmp/mainpids) &

COUNTER=1
while [ $COUNTER -lt 101 ]; do
	sleep 7
	rm -f /tmp/rospids
	(rosrun bert2_simulator object.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator pressure.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator sensors.py >> /tmp/sensorsout$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor2.py $COUNTER>> /tmp/monitor2out$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor3.py $COUNTER>> /tmp/monitor3out$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor4.py $COUNTER>> /tmp/monitor4out$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor5.py $COUNTER>> /tmp/monitor5out$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor6.py $COUNTER>> /tmp/monitor6out$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator robot.py >> /tmp/robotout$COUNTER) &
	(rosrun bert2_simulator human.py stimulus_$COUNTER >> /tmp/humanout$COUNTER) &
	(sleep 200; python src/bert2_simulator/scripts/check_code_coverage.py)
	cat /tmp/rospids | xargs kill
	
        let COUNTER=COUNTER+1 
done

cat /tmp/mainpids | xargs kill
