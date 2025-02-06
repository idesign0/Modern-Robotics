% all the inputs %

M01 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.089159; 0, 0, 0, 1];
M12 = [0, 0, 1, 0.28; 0, 1, 0, 0.13585; -1, 0, 0, 0; 0, 0, 0, 1];
M23 = [1, 0, 0, 0; 0, 1, 0, -0.1197; 0, 0, 1, 0.395; 0, 0, 0, 1];
M34 = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.14225; 0, 0, 0, 1];
M45 = [1, 0, 0, 0; 0, 1, 0, 0.093; 0, 0, 1, 0; 0, 0, 0, 1];
M56 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.09465; 0, 0, 0, 1];
M67 = [1, 0, 0, 0; 0, 0, 1, 0.0823; 0, -1, 0, 0; 0, 0, 0, 1];

G1 = diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7]);
G2 = diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393]);
G3 = diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275]);
G4 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G5 = diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219]);
G6 = diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]);

Glist = cat(3, G1, G2, G3, G4, G5, G6);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67); 
Slist = [0,         0,         0,         0,        0,        0;
         0,         1,         1,         1,        0,        1;
         1,         0,         0,         0,       -1,        0;
         0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491;
         0,         0,         0,         0,  0.81725,        0;
         0,         0,     0.425,   0.81725,        0,  0.81725];

%initialization
%thetalist = [0;-1;0;0;0;0]; % for second task
thetalist = [0;0;0;0;0;0]; % for first task
dthetalist = zeros(6,1);
taulist = zeros(6,1);
g= [0;0;-9.81];
Ftip = zeros(6,1);

T = 3; %total time
k = 1000; %steps per second
dt= T/k; %timestep
steps = T * k;

Thtamat = zeros(steps+1,6); % csv mat init

for i = 1:steps+1
    ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip, Mlist,Glist,Slist);
    [thetalist,dthetalist] = EulerStep(thetalist,dthetalist,ddthetalist,dt);  
    Thtamat(i,1:6) = thetalist';
end

    %CSV file%
    filename = '/Users/dhruvpatel29/V-REP_scenes/simulation1.csv';
    writematrix(Thtamat,filename);