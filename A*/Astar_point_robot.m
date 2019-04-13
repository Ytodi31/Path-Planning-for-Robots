clear all;
clc;

%Defining the colour map
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

resolution = 1;
width = 250;
height = 150;
x1 = 0:resolution:width;
y1 = height:-resolution:0;
[x y] = meshgrid(x1,y1);
columns = size(y1,2);
rows = size(x1,2);

%Plotting the original map
circle = (x-190).^2 +(y-130).^2 <= 225;
rectangle = (x<=100)&(x>=50)&(y<=112.5)&(y>=67.5);
ellipse = ((x-140).^2/225) + ((y-120).^2/36) <=1;
polygon = (y>15)&(37*x-20*y<6101)&(38*x+23*y<8530)&(38*x-7*y>5830)&(x>163);
poly =(2*x+19*y<1314)&(41*x+25*y>6525)&(x<164)&(y>15);
obstacles = (circle|rectangle|ellipse|polygon|poly);
output(~obstacles) = 1;
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

if start_x>251 ||start_y>151 
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


goal_x = goal_x+1;
goal_y = goal_y+1;

if goal_x>251 ||goal_y>151
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
cost_go = zeros(columns,rows);

map(~input_map) = 1;   % Mark free cells
map(obstacles)  = 2;   % Mark obstacle cells 
map(start_index)= 6;   % Mark start point
map(goal_index)= 5;    % Mark goal point

visit_status(start_index)=1; 

for i = 1:rows
    for j= 1:columns
        cost_go(j,i)=sqrt((goal(1)-j)^2 + (goal(2)-i)^2);
    end
end

cost_start = Inf(size(y));
cost_start(start_index)=0;
visit_status = zeros(size(y));
parent_node= zeros(size(y));
Q(1,1) = start_index;
Q(1,2) = cost_start(start_index);
Q(1,3) = cost_start(start_index) + cost_go(start_index);
visit_status(start_index)=1;

while length(Q(:,1))>0
map(start_index)= 6;
map(goal_index)= 5;
map(Q(:,1))=7; 
%       if (drawMapEveryTime)
%         image(0,0, map);
%         set(gca,'Ydir','normal');
%         grid on;
%         axis image;
%         drawnow;
%         axis([-1 251 -1 151]);
%       end
%     
      current_cost = Inf;
      lowest_cost = Inf;
      flush_length = length(Q(:,1));
      for i = 1:flush_length
        if Q(i,3) < lowest_cost
            current_cost = Q(i,2);
            lowest_cost = Q(i,3);
            node_position = i;
        end
      end
    
    current_index = Q(node_position,1);
    current_cost = Q(node_position,2);
    
    map(current_index)=4;
    [current_y,current_x] = ind2sub(size(y),current_index);
    t= all_actions(current_y, current_x,current_cost,y);
    total_cost = cost_start + cost_go;
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
    map(start_index)= 6;
    map(goal_index)= 5;
    map(index)= 3;
       if (drawMapEveryTime)
        image(0,0,map);
        set(gca,'Ydir','normal');
        grid on;
        axis image;
        axis([-1 251 -1 151]);
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
global total_cost
global cost_go
parent_index = sub2ind(size(y),current_y,current_x);

% MOVE RIGHT
if current_y+1 > columns
    %ignore action
else  
  current_index = sub2ind(size(y),current_y+1,current_x);
  if obstacles(current_index)==1
       % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
      Q(l+1,3)= current_cost+1+cost_go(current_index);
      total_cost(current_index)= current_cost+1+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        total_cost(current_index)= current_cost+1+cost_go(current_index);
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
       % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
      Q(l+1,3)= current_cost+sqrt(2)+cost_go(current_index);
      total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
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
    % ignore action   
elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +1;
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+1;
       Q(l+1,3)= current_cost+1+cost_go(current_index);
      total_cost(current_index)= current_cost+1+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        total_cost(current_index)= current_cost+1+cost_go(current_index);
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
       % ignore action
  elseif visit_status(current_index)==0 
      visit_status(current_index)=1;
      cost_start(current_index) = current_cost +sqrt(2);
      l=length(Q(:,1));
      parent_node(current_index)= parent_index;
      Q(l+1,1)= current_index;
      Q(l+1,2)= current_cost+sqrt(2);
       Q(l+1,3)= current_cost+sqrt(2)+cost_go(current_index);
      total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
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
       Q(l+1,3)= current_cost+1+cost_go(current_index);
      total_cost(current_index)= current_cost+1+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        total_cost(current_index)= current_cost+1+cost_go(current_index);
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
      Q(l+1,3)= current_cost+sqrt(2)+cost_go(current_index);
      total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
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
       Q(l+1,3)=  current_cost+1+cost_go(current_index);
      total_cost(current_index)= current_cost+1+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +1;
        total_cost(current_index)= current_cost+1+cost_go(current_index);
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
       Q(l+1,3)= current_cost+sqrt(2)+cost_go(current_index); 
      total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
  else  
     if current_cost+1 < cost_start(current_index)
        cost_start(current_index) = current_cost +sqrt(2);
        total_cost(current_index)= current_cost+sqrt(2)+cost_go(current_index);
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