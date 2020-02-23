PontAstar(resolution) and PointDij(resolution)

To run this code, input the resolution of required map 
ex: resolution = 1 = 1 grid per meter so original plot is unchaged. If resolution = 2, map is twice is more detailed etc.

Function then initializes the given map using half planes with respect to the give resolution.

roborics.BinaryOccupancyGrid is only used to store the values in the matrix. 

Then, a binary matrix of the map is created. A figure of the map will be shown and the user will be prompted to enter start point then goal point by clicking on the regions of the map

A color map is created to signify the actions in the GUI

A discription of the colormap is as follows:

% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - orange  - on list
% 5 - yellow - start
% 6 - green - destination
% 7 - grey - chosen route 

Finally, A* or Dijkstra search algorithms will be implemented to reach the goal node.

If a goal cannot be reached, the program will simply stop execution.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

RigidDij(radius,clearance,resolution) and RigidAstar(radius,clearance,resolution)

Similar to the point robot program, the user will be prompted to input the resolution and the start and goal points on the map.

The user will also be prompted to enter the robot's radius and clearance, then the program will use Minkowski's Sum to create the configuration space.

If the radius is 0, an error will be displayed requiring to enter a radius or use the point robot program.