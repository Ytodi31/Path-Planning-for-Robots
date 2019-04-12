clear all;
clc;
cmap = [1 1 1; ...% 1 - white      :clear cell
        0 0 0; ...% 2 - black      :Obstacle
        0 0 1; ...% 3 - red -      :Start node
        1 1 0; ...% 4 - blue  -    :Optimum path
        0 1 0; ...% 5 - green -    :Goal node
        1 0 0; ...% 6 - yellow -   :Visited nodes
	    0.5 0.5 1];%7- Violet-    :Unvisited nodes 

colormap(cmap);
drawMapEveryTime = true;

global rows
global columns
global visit_status
global cost_start
global Q
global obstacles
global parent_node
global cost_go
global total_cost

resolution =1;
width = 250;
height = 150;
x1 = 0:resolution:width;
y1 = height:-resolution:0;
[x y] = meshgrid(x1,y1);
columns = size(y1,2);
rows = size(x1,2);
x1 = 0:resolution:width;
y1 = height:-resolution:0;
output = zeros(columns,rows);

%Taking input of radmeter of rigid robot and clearance
rad = input('Please enter the radius of the Robot: ');
if rad<0
    rad = 0;
end
if isfloat(rad)
    rad= ceil(rad);
end

clearance = input('Please enter the clearance of the Robot: ');
if clearance <0
    clearance = 0;
end

if isfloat(clearance)
    clearance= ceil(clearance);
end

if clearance >0 
wall_x1 = x>=0 & x<= clearance;
wall_y1 = y>=0 & y<=clearance;
wall_x2 = x<=width & x>=width-clearance;
wall_y2 = y<=height & y>= height-clearance;
wall = wall_x1|wall_y1|wall_x2|wall_y2;
else
    wall= 0;
end
offset = ceil(rad/2)+clearance;

%Original map
circle = (x-190).^2 +(y-130).^2 <=225;
rectangle = (x<=100)&(x>=50)&(y<112.5)&(y>67.5);
ellipse = ((x-140).^2/225) + ((y-120).^2/36) <=1;
polygon = (y>15)&(37*x-20*y<6101)&(38*x+23*y<8530)&(38*x-7*y>5830)&(x>163);
poly =(2*x+19*y<1314)&(41*x+25*y>6525)&(x<164)&(y>15);

% Marking expanded circle
circle2 = (x-190).^2 +(y-130).^2 <= (15+offset)^2;
circle2= circle2-circle;

% Marking expanded ellipse
new_major = 15+offset;
new_minor = 6+offset;
ellipse2 = ((x-140).^2/(new_major)^2) + ((y-120).^2/(new_minor)^2) <=1;
ellipse2 = ellipse2 - ellipse;

% Marking expanded rectangle
c1 = (x-50).^2 +(y-67.5).^2 <=offset^2;
c2 = (x-50).^2 +(y-112.5).^2 <=offset^2;
c3 = (x-100).^2 +(y-67.5).^2 <=offset^2;
c4 = (x-100).^2 +(y-112.5).^2 <=offset^2;
r1 = (x>=50)&(x<=100)&(y>=112.5)&(y<=112.5+offset);
r2 = (x>=50)&(x<=100)&(y>=67.5-offset)&(y<=67.5);
r3 = (x>=50-offset)&(x<=50)&(y>67.5)&(y<112.5);
r4 = (x>=100)&(x<=100+offset)&(y>67.5)&(y<112.5);

% Marking expanded polygon using Minkowski sum
c5 = (x-173).^2 +(y-15).^2 <=offset^2;
c6 = (x-193).^2 +(y-52).^2 <=offset^2;
c7 = (x-170).^2 +(y-90).^2 <=offset^2;
c8 = (x-1693).^2 +(y-52).^2 <=offset^2;
c9 = (x-125).^2 +(y-56).^2 <=offset^2;
c10 = (x-150).^2 +(y-15).^2 <=offset^2;
c11 = (x-163).^2 +(y-52).^2 <=offset^2;
r5 = (y>=(15-offset)) & (y<=15)& (x>150)& (x<173);

eps = ceil(offset/2);
coeff1 = plotline1(37,-20,173,15,193,52,offset);
r6 =  (37*x-20*y>=6101)&(37*x-20*y<=coeff1)&(y>=15-eps)&(y<=52-eps);

coeff2 = plotline2(38,23,193,52,170,90,offset);
r7 =  (38*x+23*y>=8530)&(38*x+23*y<=coeff2)&(y>=52+eps)&(y<=90+eps);

coeff3 = plotline3(38,-7,170,90,163,52,offset);
r8 =  (38*x-7*y<=5830)&(38*x-7*y>=coeff3)&(y<=90+eps)&(y>=52+eps);

coeff4 = plotline3(2,19,163,52,125,56,offset);
r9 =  (2*x+19*y>=1314)&(2*x+19*y<=coeff4)&(x<=163+eps)&(x>=125-eps);

coeff5= plotline4(41,25,125,56,150,15,offset);
r10 = (41*x+25*y<=6525)&(41*x+25*y>=coeff5)&(y>=15-eps)&(y<=56-eps);

enlarged = circle2|ellipse2|c1|c2|c3|c4|r1|r2|r3|r4|c5|c6|c7|c8|c9|c10|c11|r5|r6|r7|r8|r9|r10;
obstacles = (circle|rectangle|ellipse|polygon|poly|enlarged|wall);
output(~obstacles)=1;
obstacles= flipud(obstacles);
set(gca,'Ydir','normal');
axis on;
grid on;
grid minor;
start = [];
goal = [];

%Checking for correct input
while(true)
start_x = input('Enter the X coordinates of Start node: ');
start_y = input('Enter the Y coordinates of Start node: ');
if start_x<0 ||start_y<0
disp('Start node is out of bounds of map, Retry')
    continue
end

if isfloat(start_x)
    start_x= round(start_x);
end
if isfloat(start_y)
    start_y =round(start_y);
end
start_x= start_x+1;
start_y=start_y+1;
if start_x>251 |start_y>151
    disp('Start node is out of bounds of map, Retry')
    continue
end
start = [start_y start_x];
start_index = sub2ind(size(y),start(1),start(2));
if obstacles(start_index)==1
    disp('Start is in obstacle area, retry')
    continue
end

goal_x = input('Enter the X coordinates of Goal node: ');
goal_y = input('Enter the Y coordinates of Goal node: ');
if goal_x<0 ||goal_y<0
disp('Start node is out of bounds of map, Retry')
    continue
end

if isfloat(goal_x)
    goal_x= round(goal_x);
end
if isfloat(goal_y)
    goal_y= round(goal_y);
end
goal_x=goal_x+1;
goal_y=goal_y+1;
if goal_x>251 |goal_y>151
    disp('Goal is out of bounds of map, Retry')
    continue
end
goal = [goal_y goal_x];
goal_index = sub2ind(size(y), goal(1),goal(2));
if obstacles(goal_index)==1
    disp('Goal is in obstacle area, Retry')
    continue
end
if start_index==goal_index
    disp('Goal node cannot be same as Start node, Retry')
    continue
end
break
end

input_map = ones(columns,rows);
map = ones(columns,rows);

map(~input_map) = 1;   % Mark free cells
map(obstacles)  = 2;   % Mark obstacle cells
 
map(start_index)= 6;    %Mark start point
map(goal_index)= 5;     %Mark goal point

visit_status(start_index)=1; %Array keeping track of visit status of nodes
cost_start = Inf(size(y));
cost_start(start_index)=0;
visit_status = zeros(size(y));
parent_node= zeros(size(y));    %Array recording parent of each node
Q(1,1) = start_index;
Q(1,2) = cost_start(start_index);
visit_status(start_index)=1;

while length(Q(:,1))>0
map(start_index)= 6;
map(goal_index)= 5;
map(Q(:,1))=7; 
 current_cost = Inf;
      if (drawMapEveryTime)
        image(0,0, map);
        hold on
        set(gca,'Ydir','normal');
        grid on;
        axis image;
        drawnow;
        axis([-1 251 1 151])
      end
    
      for i = 1:length(Q(:,1))
        if Q(i,2) < current_cost
            current_cost = Q(i,2);
            node_position = i;
        end
    end
    current_index = Q(node_position,1);
    current_cost = Q(node_position,2);
    map(current_index)=4;
    [current_y,current_x] = ind2sub(size(y),current_index);
    t= all_actions(current_y, current_x,current_cost,y);
    Q(node_position,:)=[];
    flag = 0;
    if current_index== goal_index
        flag =1;
        break
    end
end

%Tracing Optimal path
if length(Q(:,1))==0 &flag==0
    disp('Path to Goal node not found')
else
trace_path = shortest_path(start_index,goal_index,parent_node,y);
k= length(trace_path(:,1));
index = start_index;
while k~=0
       map(index)= 3;
       map(start_index)= 6;
       map(goal_index)= 5;
       if (drawMapEveryTime)
        image(0,0, map);
        hold on
        plot_expanded();
        hold on
        set(gca,'Ydir','normal');
        grid on;
        axis image;
        axis([-1 251 1 151]);
        drawnow;
       end
      index = sub2ind(size(y),trace_path(k,1),trace_path(k,2));
      k=k-1;      
end
end

%Funtion to find child node in all eight directions
function k = all_actions(current_y, current_x,current_cost,y)
global rows
global columns
global visit_status
global cost_start
global Q
global obstacles
global parent_node
parent_index = sub2ind(size(y),current_y,current_x);

% MOVE RIGHT
if current_y+1 > columns
     %ignore action
else  
  current_index = sub2ind(size(y),current_y+1,current_x);
  if obstacles(current_index)==1
      %ignore action 
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE UPRIGHT
if (current_x+1 > rows) | (current_y+1>columns)
     %ignore action
else  
  current_index = sub2ind(size(y),current_y+1,current_x+1);
  if obstacles(current_index)==1
        %ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE UP
if current_x+1 > rows
    %ignore action
else  
  current_index = sub2ind(size(y),current_y,current_x+1);
  if obstacles(current_index)==1
       %ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE UPLEFT
if (current_x+1 > rows) | (current_y-1<1)
     %ignore action
else  
  current_index = sub2ind(size(y),current_y-1,current_x+1);
  if obstacles(current_index)==1
      %ignore action 
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE LEFT
if current_y-1 < 1
   % ignore action
else  
  current_index = sub2ind(size(y),current_y-1,current_x);
  if obstacles(current_index)==1
     % ignore action 
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        parent_node(current_index)= parent_index;
     end
  end
end

%MOVE DOWNLEFT
if (current_x-1<1)|( current_y-1<1)
   % ignore action
else  
  current_index = sub2ind(size(y),current_y-1,current_x-1);
  if obstacles(current_index)==1
      % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE DOWN
if current_x-1 < 1
    % ignore action
else  
  current_index = sub2ind(size(y),current_y,current_x-1);
  if obstacles(current_index)==1
    % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        parent_node(current_index)= parent_index;
     end
  end
end

% MOVE DOWNRIGHT
if (current_x-1<1)|( current_y+1>columns)
    % ignore action
else  
  current_index = sub2ind(size(y),current_y+1,current_x-1);
  if obstacles(current_index)==1
       % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        parent_node(current_index)= parent_index;
     end
  end
end
k=1;
end

%Function to find the shortest path from all the explored paths
function path= shortest_path(start_index,goal_index,parent_node,y)
path= [];
current_index = goal_index;
i=1;
while(current_index~=start_index)
     [current_y,current_x] = ind2sub(size(y),current_index);
     path(i,1) = current_y;
     path(i,2) = current_x;
     current_index = parent_node(current_index);
     i=i+1;    
end
end

%Function to plot the original map against the map with minkowski sum
function p = plot_expanded()
plotcircle(190,130,15,0,2*pi,'w')
plotellipse(140,120,15,6,'w')
plot([50,100],[67.5,67.5],'w','Linewidth',2)
plot([50,100],[112.5,112.5],'w','Linewidth',2)
plot([50, 50],[67.5,112.5],'w','Linewidth',2)
plot([100,100],[67.5,112.5],'w','Linewidth',2)
plot([173,193],[15,52],'w','Linewidth',2)
plot([193,170],[52,90],'w','Linewidth',2)
plot([163,125],[52,56],'w','Linewidth',2)
plot([150,125],[15,56],'w','Linewidth',2)
plot([150,173],[15,15],'w','Linewidth',2)
plot([163,170],[52,90],'w','Linewidth',2)
hold on;
end

function plotcircle(x1,y1,r,start,limit,c)
theta = start:0.01:limit;
hold on;
plot(x1+r*cos(theta),y1+r*sin(theta),c,'Linewidth',2);
axis equal;
end

function plotellipse(x1,y1,a,b,c)
t=0:0.01:2*pi;
plot(x1+a*cos(t),y1+b*sin(t),c,'Linewidth',2);
axis equal;
end

%Functions plotline1 to plotline4 find the Minkowski sum of concave poligon
%based on user input
function min_c2 = plotline1(x_coeff, y_coeff, x1,y1,x2,y2,offset)
slope = x_coeff/y_coeff;
a=0;
b=0;
distance = 0;
min_dist= Inf;
min_c2=Inf;
centre = round(offset/2);
    for(rev = centre:-1:1)
        for(fwd = centre:1: offset)
            ssq = sqrt(fwd^2+rev^2);
            if ssq >= offset
            a = fwd;
            b = rev;
            new_x1 = x1+a;
            new_x2 = x2+a;
            new_y1 = y1-b;
            new_y2 = y2-b;
            c1 = (y_coeff*y2+x_coeff*x2);
            c2 = y_coeff*new_y1 + x_coeff*new_x1;
            distance =abs(c1-c2)/(sqrt(x_coeff^2+y_coeff^2));
                if distance>=offset
                    if distance < min_dist
                       min_dist = distance; 
                       min_c2 = c2;
                    end
                end
            end
        end
    end
    
    if min_c2== Inf
    min_c2 = round(offset*(sqrt(x_coeff^2+y_coeff^2))+c1);
end
end

function min_c2 = plotline2(x_coeff, y_coeff, x1,y1,x2,y2,offset)
slope = x_coeff/y_coeff;
a=0;
b=0;
distance = 0;
min_dist= Inf;
min_c2=Inf;
centre = round(offset/2);
    for(rev = centre:-1:1)
        for(fwd = centre:1: offset)
            ssq = sqrt(fwd^2+rev^2);
            if ssq >= offset
            a = fwd;
            b = rev;
            new_x1 = x1+a;
            new_x2 = x2+a;
            new_y1 = y1+b;
            new_y2 = y2+b;
            c1 = (y_coeff*y2+x_coeff*x2);
            c2 = y_coeff*new_y1 + x_coeff*new_x1;
            distance =abs(c1-c2)/(sqrt(x_coeff^2+y_coeff^2));
                if distance>=offset
                    if distance < min_dist
                       min_dist = distance; 
                       min_c2 = c2;
                    end
                end
            end
        end
    end   
    if min_c2== Inf
    min_c2 = round(offset*(sqrt(x_coeff^2+y_coeff^2))+c1);
end
end

function min_c2 = plotline3(x_coeff, y_coeff, x1,y1,x2,y2,offset)
min_c2= Inf;
slope = x_coeff/y_coeff;
a=0;
b=0;
min_dist= Inf;
min_c2=Inf;
distance = 0;
centre = round(offset/2);
    for(rev = centre:-1:1)
        for(fwd = centre:1: offset)
            ssq = sqrt(fwd^2+rev^2);
            if ssq >= offset
            a = fwd;
            b = rev;
            new_x1 = x1-a;
            new_x2 = x2-a;
            new_y1 = y1+b;
            new_y2 = y2+b;
            c1 = (y_coeff*y2+x_coeff*x2);
            c2 = y_coeff*new_y1 + x_coeff*new_x1;
            distance =abs(c1-c2)/(sqrt(x_coeff^2+y_coeff^2));
                if distance>=offset
                    if distance < min_dist
                       min_dist = distance; 
                       min_c2 = c2;
                    end
                end
            end
        end
    end
    
if min_c2== Inf
    min_c2 = round(offset*(sqrt(x_coeff^2+y_coeff^2))+c1);
end
end

function min_c2 = plotline4(x_coeff, y_coeff, x1,y1,x2,y2,offset)
 min_c2= Inf;
slope = x_coeff/y_coeff;
a=0;
b=0;
min_dist= Inf;
min_c2=Inf;
distance = 0;
centre = round(offset/2);
    for(rev = centre:-1:1)
        for(fwd = centre:1: offset)
            ssq = sqrt(fwd^2+rev^2);

            if ssq >= offset
            a = fwd;
            b = rev;
            new_x1 = x1-a;
            new_x2 = x2-a;
            new_y1 = y1-b;
            new_y2 = y2-b;
            c1 = (y_coeff*y2+x_coeff*x2);
            c2 = y_coeff*new_y1 + x_coeff*new_x1;
            distance =abs(c1-c2)/(sqrt(x_coeff^2+y_coeff^2));
                if distance>=offset
                    if distance < min_dist
                       min_dist = distance; 
                       min_c2 = c2;
                    end
                end
            end
        end
    end 
    
if min_c2== Inf
    min_c2 = round(offset*(sqrt(x_coeff^2+y_coeff^2))+c1);
end
end

