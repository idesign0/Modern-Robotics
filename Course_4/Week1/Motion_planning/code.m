clear;
clc;

%Datasets
edgeData = readtable("results/edges.csv");
nodeData = readtable("results/nodes.csv");


OpenSet = 1;
Closeset=[];
goal = 12;

past_cost(1) = 0; past_cost(2:size(nodeData,1))=inf;
est_total_cost(2:size(nodeData,1)) = inf;
parent(1:size(nodeData,1))=0;
pathlist=[];

while ~isempty(OpenSet)
    
    current = OpenSet(1);
    OpenSet(1) = [];
    
    % section to get pathlist
    % which happens after reaching the goal.
    if(current==goal) 
        while current ~= 1
            pathlist =[pathlist,parent(current)];
            current = pathlist(end);
        end
        pathlist = flip([goal,pathlist]);
        writematrix(pathlist,'path.csv')
        break;
    end

    nbr=[]; % getting neighbours + edge cost
    for i=1:size(edgeData,1)
        if edgeData{i,"ID2"} == current
            nbr = [nbr;edgeData{i,"x_ID1"},edgeData{i,"cost"}];
        end
    end

    nbrTcost=[]; % to get total cost associated with neighbours
        
    for i=1:size(nbr,1)
        if ~ismember(nbr(i,1),Closeset) % if nbr element is memeber of closeset, nothing will happen
             
            % tentative_past_cost = currentls past-cost + edgecost
            tentative_past_cost = past_cost(current) + nbr(i,2);
            % if new cost is optimal then n then new parent will be set up
             if tentative_past_cost < past_cost(nbr(i,1))
                 parent(nbr(i,1)) = current;
                 past_cost(nbr(i,1)) = tentative_past_cost;
                 % optimal function
                 est_total_cost(nbr(i,1)) = past_cost(nbr(i,1)) + nodeData{(nbr(i,1)),"heuristic_cost_to_go"};
             end
             %adding neighbour total cost for future shorting
             nbrTcost = [nbrTcost;nbr(i,1),est_total_cost(nbr(i,1))];
        end
    end
        %sorting based on second column
        temp = sortrows(nbrTcost,2);
        OpenSet = temp(:,1);
        Closeset = [Closeset,current];
end

