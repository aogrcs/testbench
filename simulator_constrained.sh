#!/bin/bash

cd ~/catkin_ws
(roscore & echo $! >> /tmp/mainpids) &
(sleep 5; roslaunch bert2_simulator bert2_gazebo.launch & echo $! >> /tmp/mainpids) &
(sleep 10; roslaunch bert2_moveit move_group.launch & echo $! >> /tmp/mainpids) &

COUNTER=28
while [ $COUNTER -lt 101 ]; do
	sleep 7
	rm -f /tmp/rospids
	(rosrun bert2_simulator object.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator pressure.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator sensors.py >> /tmp/sensorsout10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor2.py 10$COUNTER>> /tmp/monitor2out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor3.py 10$COUNTER>> /tmp/monitor3out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor4.py 10$COUNTER>> /tmp/monitor4out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor5.py 10$COUNTER>> /tmp/monitor5out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor6.py 10$COUNTER>> /tmp/monitor6out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator robot.py >> /tmp/robotout10$COUNTER) &
	(rosrun bert2_simulator human.py stimulus_10$COUNTER >> /tmp/humanout10$COUNTER) &
	(sleep 200; python src/bert2_simulator/scripts/check_code_coverage.py)
	cat /tmp/rospids | xargs kill
	
        let COUNTER=COUNTER+1 
done

cat /tmp/mainpids | xargs kill
