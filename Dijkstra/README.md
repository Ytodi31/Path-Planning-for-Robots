## Dijkstra Algorithm

<img src ="C:\Users\yasha\Dijkstra\Picture2.png alt="My cool logo"/>
								  
-Workspace is considered as a 8 connected space, that means now you can move the robot in up, down, left, right & diagonally between up-left, up-right, down-left and down-right directions.


								  
-Half planes and semi-algebraic sets are used to represent the Obstacles space

1. Dijkstra_point_robot
  - The program will ask for the x and y coordinates for \
   a) Start Node\
   b) Goal Node\
   These can take positive integer or float values. The program will prompt to re-enter the coordinate if the the coordinates are out of   bounds of the map or lie in the obstacle area.

  - Range:\
  Range for x coordinates - 0 to 250\
  Range for y coordinates - 0 to 150

 - The program will explore the feasible paths and show the animation of the optimal path after it is found only to optimise time.

 - For faster results,, kindly comment the following sections :\
  Lines 135 to 142\
  This will turn off the animation during node exploration.

 - For instance, the input will be as follows if the start is (0,0) and goal is (250,150):\
  Enter the X coordinates of Start node: 0\
  Enter the Y coordinates of Start node: 0\
  Enter the X coordinates of Goal node: 250\
  Enter the Y coordinates of Goal node: 150
  
  2. Dijkstra_rigid_robot
  - The rigid robot is considered to be circular in shape.
  - The enlarged obstacle space and clearance will be calculated using Minkowski sum based on user input.
  - User input :\
  The program will ask for the x and y coordinates for (a) and (b) \
  a) Start Node\
  b) Goal Node\
  c) Robot radius\
  d) Clearance

  - These can take positive integers or float values. The program will prompt to re-enter the coordinate if the the coordinates are out of bounds of the map or lie in the obstacle area.\
The obstacle area will be calculated based on the robot radius and clearance.

  - Range:\
Range for x coordinates - 0 to 250\
Range for y coordinates - 0 to 150

  - The program will explore the feasible paths and show the animation of the optimal path after it is found only to optimise time.

  - For faster results,, kindly comment the following sections :\
  Lines211 to 219\
  This will turn off the animation during node exploration.


  - For instance, the input will be as follows if the start is (3,3) and goal is (247,147) with robot radius 5mm and clearance as 2mm:\
	Please enter the radius of the Robot: 5\
	Please enter the clearance of the Robot: 2\
	Enter the X coordinates of Start node: 3\
	Enter the Y coordinates of Start node: 3\
	Enter the X coordinates of Goal node: 247\
	Enter the Y coordinates of Goal node: 147
