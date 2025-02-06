clc;clear

% Obstacles data
obstData = readtable('results/obstacles.csv');

% Define the bottom-left and top-right points of the 2D space
bottom_left=[-.5,-0.5]; top_right=[0.5,0.5];

% Phase 1: sampling
% Define the number of points to generate
num_nodes = 500;
nodes = sampler(bottom_left,top_right,num_nodes);
writematrix(nodes,'results/nodes.csv'); % nodes file

% Phase 2: Edge-Making
% Edge making
n=5;
edges = edgeCreater(nodes,n);
writematrix(edges,'results/edges.csv'); % edge file

% Phase 3: searching the graph
% A-Star
path = Astart(nodes(1,:),nodes(end,:));
writematrix(path,'results/path.csv');

% Debuging Purpose
plotter(nodes,bottom_left,top_right,path)

function points = sampler(bottom_left,top_right,num_points)
% Calculate the dimensions of the space
width = top_right(1) - bottom_left(1);
height = width;

% Generate random points within the specified space
x = [ bottom_left(1) ; bottom_left(1) + width * rand(1, num_points)'];
y = [ bottom_left(2) ; bottom_left(2) + height * rand(1, num_points)'];

% Combine x and y coordinates into a single matrix
points = [nodefilter([x y]);[ size(nodefilter([x y]),1)+1 top_right 0]];
end

function filteredpoints = nodefilter(points)
    Obst = evalin('base','obstData');
    for i=1:size(points,1)
        for j=1:size(Obst,1)
           pvecLength = sqrt((abs(points(i,1)-Obst{j,1}))^2 + (abs(points(i,2)-Obst{j,2})^2));
           if pvecLength < (Obst{j,3}/2 + 0.02)
               points(i,:) = NaN;
           end
        end
    end
    
    for i=1:size(points,1)
        if ~isnan(points(i,1))
            for j=1:size(points,1)
                if ~isnan(points(i,1)) && i ~= j
                    distance = sqrt((abs(points(i,1)-points(j,1)))^2 + (abs(points(i,2)-points(j,2))^2));
                    if distance <= 0.1
                        points(j,:) = NaN;
                    end
                end
            end
        end
    end

    points(isnan(points(:,1)),:)=[];

    sdistance=[];
    for i=1:size(points,1)
        if ~isnan(points(i,1))
            sdistance = [sdistance;sqrt((abs(points(i,1)-(0.5)))^2 + (abs(points(i,2)-(0.5))^2))];
        else
            sdistance = [sdistance;NaN];
        end
    end
    points(:,3) = sdistance;
    points(:,1:4) = [ (1:1:size(points,1))' sortrows(points,3,"descend")];
    filteredpoints = points;
end

function edges = edgeCreater(points,nbr)
    e=[];
    nbrradius = 0.3;
    for i=1:size(points,1)
       cnt=0;
       for j=1:size(points,1)
           if i~=j && cnt <= nbr
              nbrdistance = sqrt((abs(points(i,2)-points(j,2)))^2 + (abs(points(i,3)-points(j,3))^2));
              nocollision = collisionchecker(points(i,2:3),points(j,2:3),nbrdistance);
              if nbrdistance < nbrradius && nocollision
                  e = [e;i,j,nbrdistance];
                  cnt = cnt+1;
              end
           end
       end
    end
    [~,ia,~] = unique(e(:,3),'rows','stable');
    edges = e(ia,:);
end


function nocolision = collisionchecker(p1,p2,~)
    Obst = evalin('base','obstData');
    midpoint = [(p1(1)+p2(1))/2 (p1(2)+p2(2))/2];
    nocolision = true;
    for j=1:size(Obst,1)
           midpvecLength = sqrt((abs(midpoint(1)-Obst{j,1}))^2 + (abs(midpoint(2)-Obst{j,2})^2));
           if midpvecLength < (Obst{j,3}/2 + 0.02)
               nocolision = false;
               return;
           end
    end
end

function path = Astart(start,goal)

%Datasets
edgeData = readtable("results/edges.csv");
nodeData = readtable("results/nodes.csv");

OpenSet = start;
Closeset=[];

past_cost(1) = 0; past_cost(2:size(nodeData,1))=inf;
est_total_cost(2:size(nodeData,1)) = inf;
parent(1:size(nodeData,1))=0;
pathlist=[];
path = [];

while ~isempty(OpenSet)
    
    current = OpenSet(1);
    OpenSet(1) = [];
    
    % section to get pathlist
    % which happens after reaching the goal.
    if(current(1)==goal(1)) 
        while current(1) ~= start(1)
            pathlist =[pathlist,parent(current(1))];
            current(1) = pathlist(end);
        end
        path = flip([goal(1),pathlist]);
        break
    end

    nbr=[]; % getting neighbours + edge cost
    for i=1:size(edgeData,1)
        if edgeData{i,1} == current(1)
            nbr = [nbr;edgeData{i,2},edgeData{i,3}];
        end
        if edgeData{i,2} == current(1)
            nbr = [nbr;edgeData{i,1},edgeData{i,3}];
        end
    end

    nbrTcost=[]; % to get total cost associated with neighbours
        
   if ~isempty(nbr)
    for i=1:size(nbr,1)
        if ~ismember(nbr(i,1),Closeset) % if nbr element is memeber of closeset, nothing will happen
            % tentative_past_cost = currentls past-cost + edgecost
            tentative_past_cost = past_cost(current(1)) + nbr(i,2);
            % if new cost is optimal then n then new parent will be set up
             if tentative_past_cost < past_cost(nbr(i,1))
                 parent(nbr(i,1)) = current(1);
                 past_cost(nbr(i,1)) = tentative_past_cost;
                 % optimal function
                 est_total_cost(nbr(i,1)) = past_cost(nbr(i,1)) + nodeData{(nbr(i,1)),4};
             end
             %adding neighbour total cost for future shorting
             nbrTcost = [nbrTcost;nbr(i,1),est_total_cost(nbr(i,1))];
        end
    end
        %sorting based on second column
        temp = sortrows(nbrTcost,2);
        OpenSet = temp(:,1);
        Closeset = [Closeset,current(1)];
   end
end
end

function [] = plotter(points,bottom_left,top_right,path)
% Optional: Plot the points
figure;

Obst = evalin('base','obstData');
edge = evalin('base','edges');

hold on;
for i=1:size(edge,1)
   plot([points(edge(i,1),2) points(edge(i,2),2)],[points(edge(i,1),3) points(edge(i,2),3)]) 
end
scatter(points(:,2), points(:,3), 'filled','bl');
scatter(points(path,2), points(path,3), 'filled' , 'r');

viscircles([Obst{:,1},Obst{:,2}],Obst{:,3}/2)
axis([bottom_left(1) top_right(1) bottom_left(2) top_right(2)]);
grid on;
title('PRM with A-star');
xlabel('X');
ylabel('Y');
hold off;
end


