clear;


mode=4; % 1 for collapsing and 2 for stability for wikipedia assembly
% 3 for collapsing and 4 for stability for Figure 12.27 assembly
switch mode
    case 1
        % Total mass and cg. [[x y], M]
        M1 =[[25,35] 2]; M2 =[[66,42] 5];
        
        % description of each contact [b1,b2,x,y,Contact Normal,u]
        b10 =  [1, 0, 0, 0, 1.5708, 0.1];
        b12 =  [1, 2, 60, 60, 3.1416,0.5];
        b202 =  [2, 0, 60, 0, 1.5708, 0.5];
        b201 =  [2, 0, 72, 0, 1.5708, 0.5];
        
        bodyContacs = [b10;b12;b202;b201];

    case 2
        % Total mass and cg. [[x y], M]
        M1 =[[25,35] 2]; M2 =[[66,42] 10];
        
        % description of each contact [b1,b2,x,y,Contact Normal,u]
        b10 =  [1, 0, 0, 0, 1.5708, 0.5];
        b12 =  [1, 2, 60, 60, 3.1416,0.5];
        b202 =  [2, 0, 60, 0, 1.5708, 0.5];
        b201 =  [2, 0, 72, 0, 1.5708, 0.5];
        
        bodyContacs = [b10;b12;b202;b201];

    case 3
        % Total mass and cg. [[x y], M]
        M1 =[[-20,35] 1]; M2 =[[20,30] 1]; M3 =[[40,0] 20];
        
        % description of each contact [b1,b2,x,y,Contact Normal,u]
        b10 =  [1, 0, -10, 0, 1.5708, 0.1];
        b101 =  [1, 0, -20, 0, 1.5708,0.1];
        
        b20 =  [2, 0, 10, 0, 1.5708, 0.1];
        b202 =  [2, 0, 20, 0, 1.5708,0.1];

        b131 =  [1, 3, -8, 35, 2.35619, 0.1];
        b132 =  [1, 3, -12, 45, 2.35619, 0.1];
        b231 =  [2, 3, 8, 35, -0.785398, 0.1];
        b232 =  [2, 3, 12, 45, -0.785398, 0.1];
        
        bodyContacs = [b10;b101;b20;b202;b131;b132;b231;b232];

    case 4
        % Total mass and cg. [[x y], M]
        M1 =[[-20,35] 20]; M2 =[[20,30] 30]; M3 =[[40,0] 5];
        
        % description of each contact [b1,b2,x,y,Contact Normal,u]
        b10 =  [1, 0, -10, 0, 1.5708, 0.5];
        b101 =  [1, 0, -20, 0, 1.5708,0.5];
        
        b20 =  [2, 0, 10, 0, 1.5708, 0.5];
        b202 =  [2, 0, 20, 0, 1.5708,0.5];

        b131 =  [1, 3, -8, 35, 2.35619, 0.5];
        b132 =  [1, 3, -12, 45, 2.35619, 0.5];
        b231 =  [2, 3, 8, 35, -0.785398, 0.5];
        b232 =  [2, 3, 12, 45, -0.785398, 0.5];
    
        bodyContacs = [b10;b101;b20;b202;b131;b132;b231;b232];

    otherwise
        error('An unexpected condition occurred.');
        % always trap the unexpected input!!!
end

F = [];
% Body contacs and wrenches
for i=1:size(bodyContacs,1)
    [f1,f2]=frictioncone(bodyContacs(i,:));
    F = [F,f1',f2'];
end

if mode==1 || mode==2
    % external forces
    Fext = [[-M1(1)*M1(3),0,-M1(3)]',[-M2(1)*M2(3),0,-M2(3)]'];
    f = ones(1,8);
    A = -eye(8);
    b = -ones(1,8);
    beq = -Fext(:,2);
    Aeq = F;
    
    if mode==1
        fprintf('Collapsing assembly :')
        k = linprog(f,A,b,Aeq,beq)
    else
        fprintf('Assembly that can continue to stand:')
        k = linprog(f,A,b,Aeq,beq)
    end
else
    % external forces
    Fext = [[M1(1)*M1(3),0,-M1(3)]',[-M2(1)*M2(3),0,-M2(3)]',[-M3(1)*M3(3),0,-M3(3)]'];
    n=16;
    f = ones(1,n);
    A = -eye(n);
    b = -ones(1,n);
    beq = -Fext(:,2);
    Aeq = -F;
    
    if mode==3
        fprintf('Collapsing assembly :')
        k = linprog(f,A,b,Aeq,beq)
    else
        fprintf('Assembly that can continue to stand:')
        k = linprog(f,A,b,Aeq,beq)
    end
end

%for wrentches on friction cone sides
function [ft1,ft2] = frictioncone(b)
    x=b(3);y=b(4);
    cNr = b(5);
    u = b(6);
    
    fn = [cos(cNr);sin(cNr)];
    Rn = [0 1;-1 0];
    
    uf1 = Rn*u*fn;
    uf2 = -uf1;

    ft = (cos(atan(u)))*(fn + uf1);
    ft1 = [ft(2)*x-ft(1)*y,ft(1),ft(2)];
    
    ft = (cos(atan(u)))*(fn + uf2);
    ft2 = [ft(2)*x-ft(1)*y,ft(1),ft(2)];
end

% Figure 12.27