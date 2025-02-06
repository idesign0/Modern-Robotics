clc;

% Odometry
dt = 0.01;
r = 0.0475; 
l= 0.47/2;
w= 0.15;
F = r/4*[-1/l+w 1/l+w 1/l+w -1/l+w; 1 1 1 1;-1 1 -1 1]; % {du -> Vb}
F_6 = [zeros(2,4) ; F ; zeros(1,4)];


% Initial config and Gripper State.
InitialConfig = [-0.75959, -0.47352, 0.058167, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, 1.506, -1.3338, 1.5582, 1.6136, 0]; % global, joint, wheel value
Tbo= [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
Moe= [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]]; % Arm Body Screw Axis
    
currentpos = InitialConfig;
currentpostrack = InitialConfig; % for joint and wheel sequence file

% ref trajectory Generation for a Controller
Tsb = [cos(currentpos(1)) -sin(currentpos(1)) 0 currentpos(2);sin(currentpos(1)) cos(currentpos(1)) 0 currentpos(3); 0 0 1 0.0963; 0 0 0 1]; % {s} -> {b}
Toe= FKinBody(Moe,Blist,currentpos(4:8)'); % {0} -> end effector {e}
Tse = Tsb*Tbo*Toe;  % {s} -> {e}
refTrajectory = TrajectoryGenerator(Tse);

% Loop for Control algorithm

Xerrtrack = [];
for i=1:size(refTrajectory,1)-2
     
     % Base and Manipulator Jacobians and Current Configurations (updatating with each iteration)
    
     Tsbc = [cos(currentpos(1)) -sin(currentpos(1)) 0 currentpos(2);sin(currentpos(1)) cos(currentpos(1)) 0 currentpos(3); 0 0 1 0.0963; 0 0 0 1]; % {s} -> {b}
     Toe= FKinBody(Moe,Blist,currentpos(4:8)'); % {0} -> end effector {e}
     Tsec = Tsbc*Tbo*Toe; % {s} -> {e}
     
     je_arm = JacobianBody(Blist,currentpos(4:8)');
     je_base = Adjoint(TransInv(Toe)*TransInv(Tbo))*F_6;
     Je = [je_base je_arm];
     
     % Control Vec and Xerr
     [controlVec,Xerr] = Feedback_Control(refTrajectory(i,:),refTrajectory(i+2,:),Tsec,currentpos,dt,Je);
     Xerrtrack = [Xerrtrack;Xerr'];

     % Updating Next Position
     currentpos = Nextstep(controlVec,currentpos,dt);
     currentpostrack = [currentpostrack;currentpos];
end
 
plot(Xerrtrack);
legend('Wx','Wy','Wz','X','Y','Z')
csvwrite('output.csv',currentpostrack);

csvwrite('Xerr.csv',Xerrtrack);
