function [] = PointDij(resolution)
A = abs(resolution);

% Create obstacle space 
map = robotics.BinaryOccupancyGrid(250*A,150*A,1); %defining a 250x150 space
                                                    ... used only to store information about the obstacle space


% define square
xs1o=(50*A)-0.5:0.1/A:(100*A)+0.5;
ys1o=(37.5)*ones(size(xs1o))*A;

xs1=(50)*A:0.1/A:(100)*A;
ys1=(112.5)*ones(size(xs1))*A;
xs1=xs1';
ys1=ys1';
setOccupancy(map, [xs1 ys1], 1)

ys2o=(37.5*A):0.1/A:(82.5*A)+1;
xs2o=50*ones(size(ys2o))*A-0.5;

ys2=(67.5)*A:0.1/A:(112.5)*A;
xs2=(50)*ones(size(ys2))*A;
ys2=ys2';
xs2=xs2';
setOccupancy(map, [xs2 ys2], 1)

xs3o=(50*A)-0.5:0.1/A:(100*A)+0.5;
ys3o=82.5*ones(size(xs3o))*A+1;

xs3=(50)*A:0.1/A:(100)*A;
ys3=(67.5)*ones(size(xs3))*A;
ys3=ys3';
xs3=xs3';
setOccupancy(map, [xs3 ys3], 1)

ys4o=(37.5*A):0.1/A:(82.5*A)+1;
xs4o=100*ones(size(ys4o))*A+0.5;

ys4=(67.5)*A:0.1/A:(112.5)*A;
xs4=(100)*ones(size(ys4))*A;
ys4=ys4';
xs4=xs4';
setOccupancy(map, [xs4 ys4], 1) %done with square


t=-pi:0.01/A:pi;
x0=140*A; % x0,y0 ellipse centre coordinates
y0=120*A;
a=15*A; % horizontal radius
b=6*A; % vertical radius

xeo=x0+0.5+a*cos(t); %offset 0.5
yeo=(y0+0.5-90*A)+b*sin(t); % offset 0.5

a=15;
b=6;
xe=x0+((a)*A*cos(t));
ye=y0+((b)*A*sin(t));
xe=xe';
ye=ye';
setOccupancy(map, [xe ye], 1) %done plotting ellipse
% % plot(xeo,yeo)
% hold on



th = 0:pi/(A*1000):2*pi;
xco = (15*A) * cos(th) + (190*A+0.5); %original coordinates for reference (no clearance)
yco = (15*A) * sin(th) + (20*A+0.5);
xc = ((15)*A) * cos(th) + 190*A; %coordinates with clearance and resolution 
yc = ((15)*A) * sin(th) + 130*A;
xc=xc';
yc=yc';
setOccupancy(map, [xc yc], 1) %done plotting circle



xp1o=125*A:0.01/A:150*A;
yp1o=(41/25)*xp1o-111*A;

xp1=(125-1.5)*A:0.01/A:(150)*A;
yp1=-(41/25)*xp1+(261)*A;
xp1=xp1';
yp1=yp1';
setOccupancy(map, [xp1 yp1], 1)


xp2o=(125)*A:0.01/A:(163)*A;
yp2o=(2/19)*xp2o+((1536/19))*A;

xp2=(125-1.5)*A:0.01/A:(163)*A;
yp2=-(2/19)*xp2+((1314/19))*A;
xp2=xp2';
yp2=yp2';
setOccupancy(map, [xp2 yp2], 1)

xp3o=(163)*A:0.01/A:(170)*A;
yp3o=-(38/7)*xp3o+(6880/7)*A;

xp3=(163)*A:0.01/A:(170)*A;
yp3=(38/7)*(xp3)-((5830/7))*A;
xp3=xp3';
yp3=yp3';
setOccupancy(map, [xp3 yp3], 1)

xp4o=170*A:0.01/A:193*A;
yp4o=(38/23)*xp4o-(5080/23)*A;

xp4=(170)*A:0.01/A:(193+0.15)*A;
yp4=-(38/23)*(xp4+1)+((8530/23))*A;
xp4=xp4';
yp4=yp4';
setOccupancy(map, [xp4 yp4], 1)

xp5o=173*A:0.01/A:193*A;
yp5o=-(37/20)*xp5o+(9101/20)*A;

xp5=(173-0.5)*A:0.01/A:(193+0.15)*A;
yp5=(37/20)*(xp5+1)-((6101/20)+1)*A;
xp5=xp5';
yp5=yp5';
setOccupancy(map, [xp5 yp5], 1)

xp6o=150*A:0.01/A:173*A;
yp6o=(135*ones(size(xp6o)))*A;

xp6=(150)*A:0.01/A:(173+0.5)*A;
yp6=((15)*ones(size(xp6)))*A;
xp6=xp6';
yp6=yp6';
setOccupancy(map, [xp6 yp6], 1) %done plotting polygon


map = occupancyMatrix(map);

map = imfill(map,'holes');



input_map = map;

figure
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
imshow(~map)
axis on
hold on
plot(xco,yco,xeo,yeo,xs1o,ys1o,xs2o,ys2o,xs3o,ys3o,xs4o,ys4o,xp1o,yp1o,...
    xp2o,yp2o,xp3o,yp3o,xp4o,yp4o,xp5o,yp5o,xp6o,yp6o)%plot shapes with no clearance

[x_coords,y_coords] = ginput(2); %input location of start point then goal point
x_coords = round(x_coords)
y_coords = round(y_coords)

close all

%% Dijkstra Algorithm initialization
% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - orange  - on list
% 5 - yellow - start
% 6 - green - destination
% 7 - grey - chosen route

cmap = [1 1 1; 
    0 0 0; 
    1 0 0; 
    1 165/255 0; 
    1 1 0; 
    0 1 0; 
    0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
mapAnim = true;

[nrows, ncols] = size(input_map); 

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells 1 is white
map(input_map)  = 2;   % Mark obstacle cells 2 is black

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), y_coords(1), x_coords(1));   % 16
dest_node  = sub2ind(size(map), y_coords(2),  x_coords(2)); %88
map(start_node) = 5; %5 is green
map(dest_node)  = 6; %6 is yellow

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = -1;

while true
    % Draw current map
    map(start_node) = 5; 
    map(dest_node) = 6; 
    numExpanded=numExpanded+1;
    % make mapAnim = false if search animation is not needed 
    if (mapAnim)
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:)); %current is node of min_dist in terms of lin ind
    if ((current == dest_node) || isinf(min_dist))
        break;
    end
    
    % Update map
    map(current) = 3;         % mark current node as visited 3 is red
    curr_dist = distanceFromStart(current);
    distanceFromStart(current) = Inf; % remove this node from further consideration
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    reqd_neigh=[[i-1,j],[i+1,j],[i,j-1],[i,j+1],[i-1,j+1],[i+1,j+1],[i+1,j-1],[i-1,j-1]];
    for t=1:8
        element=reqd_neigh(2*t-1:2*t); %1,2...3,4...5,6 etc
        m=element(1); %y
        p=element(2); %x
        if m>0 && p>0 && m<nrows+1 && p<ncols+1
            if map(m,p)==1 % indicates clear cell
                if distanceFromStart(m,p)>curr_dist+1
                    distanceFromStart(m,p)=curr_dist+1;
                    parent(m,p)=current; %records ind of child in cell of parent matix ex m = 120 p = 47 is parent of 7021 (ind of 121 47)
                    map(m,p)=4; %4 is blue (on list)
                end
            elseif map(m,p)==6 % indicates destination 6 is yellow goal
                    distanceFromStart(m,p)=curr_dist+1;
                    parent(m,p)=current;
            end
        end
    end
end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1  
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
        map(route(k)) = 7;
        pause(0.01);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end


end