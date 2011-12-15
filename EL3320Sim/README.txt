Instructions:
1.Generate files map.txt and traj.txt with generateMap() in MATLAB.
2.Generate simoutput.txt using the following commands in a terminal.
	a)With the basic configuration:
	java -cp EL3320Sim-20090311.jar FileGenerator -c default_settings.txt
	b)With a customized configuration:
	java -cp EL3320Sim-20090311.jar FileGenerator -c default_settings.txt -e exp_settings.txt
3.Display tha animation with displaySimOutput('simoutput.txt','map.txt') in MATLAB.


MATLAB files:
1.generateMap(): generates files map.txt and traj.txt by selcting the points on the screen
2.drawLandmarkMap('map.txt'): displays the file map.txt which contains the landmarks
3.displaySimOutput('simoutput.txt','map.txt'): displays an animation with the landmarks, the true position of the robot and the odometry information

Jar file:
1.EL3320Sim-20090311.jar: simulator which generates the file simoutput.txt

txt files:
1.default_settings.txt: basic configuration
2.exp_settings.txt: modifies the simulation parameters without having to change the code
3.map.txt: specifies where the landmarks in the world are and what their ID is.  Each line in the map file has the following format: ID x y
4.traj.txt: specifies in sequence target actions along the trajectory.
	a)Stand still and measure for a certain number of iteration
	0 NUM_ITERS
	b)Goto a certain position (x,y) with a certain tolerance [m]
	1 X Y TOL
	c)Turn to a certain angle(theta) with a certain tolerance [rad]
	2 THETA TOL
	d)Jump to a certain location to simulate robot kidnapping
	3 X Y THETA
5.simoutput.txt: contains the following information on each line in the file (in this order).
	a)time
	b)odomX odomY odomTheta encoderTickRight encoderTickLeft
	c)trueX trueY trueTheta
	d)N id1 bearing1 range1 ... idN bearingN rangeN
6.map_o3.txt: map file of Lab1
7.so_o3_ie.txt: simulation file of Lab1
