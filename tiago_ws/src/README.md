1. To launch Gazebo and world :
	ros2 launch tiago_gazebo tiago_gazebo.launch.py

2. To change the world to spwan in, go to: pal_gazebo_worlds/launch/pal_gazebo.launch.py

3. To launch navigation with map:  ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=false rviz:=True #slam:=True

	- Navigation is then launched from here : /home/lar/tiago_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_bringup/launch/nav_bringup.launch.py
	  Map is hardcoded in there but its not important... works...

4. To set new map just change it in pmb2_maps package and build it

5. Spawning coordinates change: in tiago_spawn.launch.py
