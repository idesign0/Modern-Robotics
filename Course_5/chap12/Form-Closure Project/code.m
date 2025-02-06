f = [1,1,1,1];
A = [[-1,0,0,0]; [0,-1,0,0]; [0,0,-1,0]; [0,0,0,-1]];
b = [-1,-1,-1,-1];

% Test
% Figure 12.7 in book.
%F = [[0,0,-1,2]; [-1,0,1,0]; [0,-1,0,1]];  % Spans R fully  
%F = [[0,0,0,2]; [-1,0,1,0]; [0,-1,0,-1]]; % not Spanning Fully

%Figure 12.16 in book
%assuming cube lendth 4 and force are at length 3 and 1. 
f1 = [-3;1;0];
f2 = [3;0;1];
f3 = [1;-1;0];
f4 = [-1;0;-1];

F1 = [f1 f2 f3 f4];

Aeq1 = F1;
beq = [0,0,0];
k1 = linprog(f,A,b,Aeq1,beq); %spans whole R space - form closure

% shifting one F to top right

f4 = [-3;0;-1];
F2 = [f1 f2 f3 f4];

Aeq2 = F2;
beq = [0,0,0];
k2 = linprog(f,A,b,Aeq2,beq); % can't span whole R space