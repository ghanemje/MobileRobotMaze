function [] = RigidDij(radius,clearance,resolution)
A = abs(resolution);
C = abs(clearance);
rs = abs(radius);

assert(rs ~=0,'Radius of rigid robot cannot be zero. Enter radius or use PointDij instead.' )

% Create obstacle space 
map = robotics.BinaryOccupancyGrid(250*A,150*A,1); %defining a 250x150 space
                                                    ... used only to store information about the obstacle space


% create boundary around map edges equal to the clearance and radius of
% robot
ybr1 = (C+rs)*A:0.1/A:(150-C-rs)*A;
xbr1 = (C+rs)*ones(size(ybr1))*A;
xbr1=xbr1';
ybr1=ybr1';
setOccupancy(map, [xbr1 ybr1], 1)

xbr2 = (C+rs)*A:0.1/A:(250-C-rs)*A;
ybr2 = (C+rs)*ones(size(xbr2))*A;
xbr2=xbr2';
ybr2=ybr2';
setOccupancy(map, [xbr2 ybr2], 1)

ybr3 = (C+rs)*A:0.1/A:(150-C-rs)*A;
xbr3 = (250-C-rs)*ones(size(ybr3))*A;
xbr3=xbr3';
ybr3=ybr3';
setOccupancy(map, [xbr3 ybr3], 1)

xbr4 = (C+rs)*A:0.1/A:(250-C-rs)*A;
ybr4 = (150-C-rs)*ones(size(xbr4))*A;
xbr4=xbr4';
ybr4=ybr4';
setOccupancy(map, [xbr4 ybr4], 1)
    
% define square
xs1o=(50*A)-0.5:0.1/A:(100*A)+0.5;
ys1o=(37.5)*ones(size(xs1o))*A;

xs1=(50-C)*A:0.1/A:(100+C)*A;
ys1=(112.5+C)*ones(size(xs1))*A;
t=-pi-1:0.01/A:pi;
if (rs~=0)
    for i = 1:length(xs1)
        xc = ((rs+C)*A) * cos(t) + xs1(i); %coordinates with clearance and resolution 
        yc = ((rs+C)*A) * sin(t) + ys1(i);
        xc=xc';
        yc=yc';
        indicesxh = find(xc>250*A); %ignore any point that lies outsite the boundary
        xc(indicesxh) = 140*A;
        indicesxL = find(xc<0);
        xc(indicesxL) = 140*A;
        indicesyh = find(yc>150*A);
        yc(indicesyh) = 20*A;
        indicesyL = find(yc<0);
        yc(indicesyL) = 20*A;
        setOccupancy(map, [xc yc], 1)
    end
end
xs1=xs1';
ys1=ys1';
setOccupancy(map, [xs1 ys1], 1)


ys2o=(37.5*A):0.1/A:(82.5*A)+1;
xs2o=50*ones(size(ys2o))*A-0.5;

ys2=(67.5-C)*A:0.1/A:(112.5+C)*A;
xs2=(50-C)*ones(size(ys2))*A;
if (rs~=0)
    for i = 1:length(xs2)
        xc = ((rs+C)*A) * cos(t) + xs2(i); %coordinates with clearance and resolution 
        yc = ((rs+C)*A) * sin(t) + ys2(i);
        xc=xc';
        yc=yc';
        indicesxh = find(xc>250*A); %ignore any point that lies outsite the boundary
        xc(indicesxh) = 140*A;
        indicesxL = find(xc<0);
        xc(indicesxL) = 140*A;
        indicesyh = find(yc>150*A);
        yc(indicesyh) = 20*A;
        indicesyL = find(yc<0);
        yc(indicesyL) = 20*A;
        setOccupancy(map, [xc yc], 1)
    end
end
ys2=ys2';
xs2=xs2';
setOccupancy(map, [xs2 ys2], 1)

xs3o=(50*A)-0.5:0.1/A:(100*A)+0.5;
ys3o=82.5*ones(size(xs3o))*A+1;

xs3=(50-C)*A:0.1/A:(100+C)*A;
ys3=(67.5-C)*ones(size(xs3))*A;
if (rs~=0)
    for i = 1:length(xs3)
        xc = ((rs+C)*A) * cos(t) + xs3(i); %coordinates with clearance and resolution 
        yc = ((rs+C)*A) * sin(t) + ys3(i);
        xc=xc';
        yc=yc';
        indicesxh = find(xc>250*A); %ignore any point that lies outsite the boundary
        xc(indicesxh) = 140*A;
        indicesxL = find(xc<0);
        xc(indicesxL) = 140*A;
        indicesyh = find(yc>150*A);
        yc(indicesyh) = 20*A;
        indicesyL = find(yc<0);
        yc(indicesyL) = 20*A;
        setOccupancy(map, [xc yc], 1)
    end
end
ys3=ys3';
xs3=xs3';
setOccupancy(map, [xs3 ys3], 1)

ys4o=(37.5*A):0.1/A:(82.5*A)+1;
xs4o=100*ones(size(ys4o))*A+0.5;

ys4=(67.5-C)*A:0.1/A:(112.5+C)*A;
xs4=(100+C)*ones(size(ys4))*A;
if (rs~=0)
    for i = 1:length(xs4)
        xc = ((rs+C)*A) * cos(t) + xs4(i); %coordinates with clearance and resolution 
        yc = ((rs+C)*A) * sin(t) + ys4(i);
        xc=xc';
        yc=yc';
        indicesxh = find(xc>250*A); %ignore any point that lies outsite the boundary
        xc(indicesxh) = 140*A;
        indicesxL = find(xc<0);
        xc(indicesxL) = 140*A;
        indicesyh = find(yc>150*A);
        yc(indicesyh) = 20*A;
        indicesyL = find(yc<0);
        yc(indicesyL) = 20*A;
        setOccupancy(map, [xc yc], 1)
    end
end
ys4=ys4';
xs4=xs4';
setOccupancy(map, [xs4 ys4], 1) %done with square


x0=140*A; % x0,y0 ellipse centre coordinates
y0=120*A;
a=15*A; % horizontal radius
b=6*A; % vertical radius

xeo=x0+0.5+a*cos(t); %offset 0.5
yeo=(y0+0.5-90*A)+b*sin(t); % offset 0.5

a=15;
b=6;
xe=x0+((a+C)*A*cos(t));
ye=y0+((b+C)*A*sin(t));
indicesxh = find(xe>250*A); %ignore any point that lies outsite the boundary
xe(indicesxh) = 140;
indicesxL = find(xe<0);
xe(indicesxL) = 140;
indicesyh = find(ye>150*A);
ye(indicesyh) = 20;
indicesyL = find(ye<0);
ye(indicesyL) = 20;
if (rs~=0)
    for i = 1:length(xe)
        xm = ((rs+C)*A) * cos(t) + xe(i); 
        ym = ((rs+C)*A) * sin(t) + ye(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A); %ignore any point that lies outsite the boundary
        xm(indicesxh) = 140;
        indicesxL = find(xm<0);
        xm(indicesxL) = 140;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 20;
        indicesyL = find(ym<0);
        ym(indicesyL) = 20;
        setOccupancy(map, [xm ym], 1)
    end
end
xe=xe';
ye=ye';
setOccupancy(map, [xe ye], 1) %done plotting ellipse
% % plot(xeo,yeo)
% hold on



% t = 0:pi/(A*1000):2*pi;
xco = (15*A) * cos(t) + (190*A+0.5); %original coordinates for reference (no clearance)
yco = (15*A) * sin(t) + (20*A+0.5);

xc = ((15+C)*A) * cos(t) + 190*A; %coordinates with clearance and resolution 
yc = ((15+C)*A) * sin(t) + 130*A;

indicesxh = find(xc>250*A); %ignore any point that lies outsite the boundary
xc(indicesxh) = 190;
indicesxL = find(xc<0);
xc(indicesxL) = 190;
indicesyh = find(yc>150*A);
yc(indicesyh) = 120;
indicesyL = find(yc<0);
yc(indicesyL) = 120;
if (rs~=0)
    for i = 1:length(xc)
        xm = ((rs+C)*A) * cos(t) + xc(i); 
        ym = ((rs+C)*A) * sin(t) + yc(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A);
        xm(indicesxh) = 190;
        indicesxL = find(xm<0);
        xm(indicesxL) = 190;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 120;
        indicesyL = find(ym<0);
        ym(indicesyL) = 120;
        setOccupancy(map, [xm ym], 1)
    end
end
xc=xc';
yc=yc';
setOccupancy(map, [xc yc], 1) %done plotting circle


%plot polygon using half planes

xp1o=125*A:0.01/A:150*A;
yp1o=(41/25)*xp1o-111*A;

xp1=(125-C-1.5)*A:0.01/A:(150)*A;
yp1=-(41/25)*xp1+(261-C)*A;
if (rs~=0)
    for i = 1:length(xp1)
        xm = ((rs+C)*A) * cos(t) + xp1(i); 
        ym = ((rs+C)*A) * sin(t) + yp1(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A); %ignore any point that lies outsite the boundary
        xm(indicesxh) = 140;
        indicesxL = find(xm<0);
        xm(indicesxL) = 140;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 20;
        indicesyL = find(ym<0);
        ym(indicesyL) = 20;
        setOccupancy(map, [xm ym], 1)
    end
end
xp1=xp1';
yp1=yp1';
setOccupancy(map, [xp1 yp1], 1)


xp2o=(125)*A:0.01/A:(163)*A;
yp2o=(2/19)*xp2o+((1536/19))*A;

xp2=(125-C-1.5)*A:0.01/A:(163)*A;
yp2=-(2/19)*xp2+((1314/19)+C)*A;
if (rs~=0)
    for i = 1:length(xp2)
        xm = ((rs+C)*A) * cos(t) + xp2(i); 
        ym = ((rs+C)*A) * sin(t) + yp2(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A); %ignore any point that lies outsite the boundary
        xm(indicesxh) = 140;
        indicesxL = find(xm<0);
        xm(indicesxL) = 140;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 20;
        indicesyL = find(ym<0);
        ym(indicesyL) = 20;
        setOccupancy(map, [xm ym], 1)
        setOccupancy(map, [xm ym], 1)
    end
end
xp2=xp2';
yp2=yp2';
setOccupancy(map, [xp2 yp2], 1)

xp3o=(163)*A:0.01/A:(170)*A;
yp3o=-(38/7)*xp3o+(6880/7)*A;

xp3=(163)*A:0.01/A:(170)*A;
yp3=(38/7)*(xp3)-((5830/7)-C)*A;
if (rs~=0)
    for i = 1:length(xp3)
        xm = ((rs+C)*A) * cos(t) + xp3(i); 
        ym = ((rs+C)*A) * sin(t) + yp3(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A); %ignore any point that lies outsite the boundary
        xm(indicesxh) = 140;
        indicesxL = find(xm<0);
        xm(indicesxL) = 140;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 20;
        indicesyL = find(ym<0);
        ym(indicesyL) = 20;
        setOccupancy(map, [xm ym], 1)
        setOccupancy(map, [xm ym], 1)
    end
end
xp3=xp3';
yp3=yp3';
setOccupancy(map, [xp3 yp3], 1)

xp4o=170*A:0.01/A:193*A;
yp4o=(38/23)*xp4o-(5080/23)*A;

xp4=(170)*A:0.01/A:(193+C+0.15)*A;
yp4=-(38/23)*(xp4+1)+((8530/23)+C)*A;
if (rs~=0)
    for i = 1:length(xp4)
        xm = ((rs+C)*A) * cos(t) + xp4(i); 
        ym = ((rs+C)*A) * sin(t) + yp4(i);
        xm = xm';
        ym = ym';
        setOccupancy(map, [xm ym], 1)
    end
end
xp4=xp4';
yp4=yp4';
setOccupancy(map, [xp4 yp4], 1)

xp5o=173*A:0.01/A:193*A;
yp5o=-(37/20)*xp5o+(9101/20)*A;

xp5=(173-0.5)*A:0.01/A:(193+C+0.15)*A;
yp5=(37/20)*(xp5+1)-((6101/20)+C+1)*A;
if (rs~=0)
    for i = 1:length(xp5)
        xm = ((rs+C)*A) * cos(t) + xp5(i); 
        ym = ((rs+C)*A) * sin(t) + yp5(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A); %ignore any point that lies outsite the boundary
        xm(indicesxh) = 140*A;
        indicesxL = find(xm<0);
        xm(indicesxL) = 140*A;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 20*A;
        indicesyL = find(ym<0);
        ym(indicesyL) = 20*A;
        setOccupancy(map, [xm ym], 1)
    end
end
xp5=xp5';
yp5=yp5';
setOccupancy(map, [xp5 yp5], 1)

xp6o=150*A:0.01/A:173*A;
yp6o=(135*ones(size(xp6o)))*A;

xp6=(150)*A:0.01/A:(173+0.5)*A;
yp6=((15-C)*ones(size(xp6)))*A;
indicesxh = find(xp6>250*A);
xp6(indicesxh) = 190;
indicesxL = find(xp6<0);
xp6(indicesxL) = 190;
indicesyh = find(yp6>150*A);
yp6(indicesyh) = 120;
indicesyL = find(yp6<0);
yp6(indicesyL) = 120;
if (rs~=0)
    for i = 1:length(xp6)
        xm = ((rs+C)*A) * cos(t) + xp6(i); 
        ym = ((rs+C)*A) * sin(t) + yp6(i);
        xm = xm';
        ym = ym';
        indicesxh = find(xm>250*A);
        xm(indicesxh) = 190*A;
        indicesxL = find(xm<0);
        xm(indicesxL) = 190*A;
        indicesyh = find(ym>150*A);
        ym(indicesyh) = 120*A;
        indicesyL = find(ym<0);
        ym(indicesyL) = 120*A;
        setOccupancy(map, [xm ym], 1)
    end
end
xp6=xp6';
yp6=yp6';
setOccupancy(map, [xp6 yp6], 1) %done plotting polygon


map = occupancyMatrix(map);

map = imfill(map,[1 1; 20*A 190*A; 60*A 75*A; 30*A 140*A; 97*A 178*A]);


input_map = map;

figure
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
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

