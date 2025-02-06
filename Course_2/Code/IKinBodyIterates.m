function [thetalist, success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
clear,clc;
% All the modified code includes Signature - DHP and related comments.
% Example Inputs:

Blist = [[0; 1; 0; 0.191; 0; 0.817], [0; 0; 1; 0.095; -0.817; 0], [0; 0; 1; 0.095; -0.425; 0],[0; 0; 1; 0.095; 0; 0],[0; -1; 0; -0.082; 0; 0],[0; 0; 1; 0; 0; 0]]
M = [[-1, 0, 0, 0.817]; [0, 0, 1, 0.191]; [0, 1, 0, -0.006]; [0, 0, 0, 1]]
T = [[0, 1, 0, -0.5]; [0, 0, -1, 0.1]; [-1, 0, 0, 0.1]; [0, 0, 0, 1]]
thetalist0 = [3*pi/4; -pi/4 ; pi/3 ; 0; 0; 0]
eomg = 0.001
ev = 0.0001

thetalist = thetalist0;
i = 0;
maxiterations = 20;
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    
% DHP - Report Variable
% used {} to package each matrix into single cell.
JointMat = [[0,0,0,0,0,0];thetalist'];
Report = {i,thetalist,FKinBody(M,Blist,thetalist),Vb,norm(Vb(1: 3)),norm(Vb(4: 6))};
IteratesReport(Report); % Defined at end of the code.
% DHP

while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    % DHP
        JointMat = [JointMat;thetalist'];
        Report = {i,thetalist,FKinBody(M,Blist,thetalist),Vb,norm(Vb(1: 3)),norm(Vb(4: 6))};
        IteratesReport(Report);
    % DHP
end
    %CSV file%
    filename = 'iterates.csv';
    writematrix(JointMat,filename);

    success = ~ err;
end

% DHP - Added function - IKinBodyIterates ***
function IteratesReport(Report)

    disp(['Iteration : ', num2str(Report{1})]);   
    disp(['Joint Vector : ', num2str(Report{2}')]);
        
    disp(['Error Twist V_b : ', num2str(Report{4}')]);
    disp(['angular error magnitude ∣∣omega_b∣∣ : ', num2str(Report{5}')]);
    disp(['angular error magnitude ∣∣v_b∣∣ : ', num2str(Report{6}')]);

    disp('SE(3) end−effector config : ');
    disp(Report{3}); 
end
