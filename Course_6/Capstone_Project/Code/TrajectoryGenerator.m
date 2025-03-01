function refTrajectory =  TrajectoryGenerator(InitialConfig)

    % Given Trajectories to Track.
    % InitialConfig overridern according to final milestonme page requirement
    InitialConfig = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
    Tse_initial = InitialConfig;   
    Tce_standoff_grasp = [0.707 0.707 0 1; 0.707 -0.707 0 1; 0 0 -1 0.125; 0 0 0 1];
    %Tce_grasp = [-1 0 0 1; 0 1 0 0; 0 0 -1 0.025; 0 0 0 1]; % old task
    Tce_grasp = [0.707 0.707 0 1; 0.707 -0.707 0 1; 0 0 -1 0.025; 0 0 0 1]; % new Task
    T_mid = [0 0 1 0; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
    %T_mid = [-1 0 0 0; 0 1 0 0; 0 0 -1 0.075; 0 0 0 1];
    Tce_standoff_goal = [1 0 0 2; 0 -1 0 -2; 0 0 -1 0.125; 0 0 0 1];
    %Tsc_goal = [0 1 0 0; 1 0 0 -1; 0 0 -1 0.025; 0 0 0 1]; % old Task
    Tsc_goal = [1 0 0 2; 0 -1 0 -2; 0 0 -1 0.025; 0 0 0 1]; % new Task


    % Timing for each segments     
    dt =0.01;
    T = 3;
    K = 1;
    N = T*K/dt;

    % output track and file
    refTrajectory=[];
    refTrajectory = [refTrajectory;refTrajectoryfun(Tse_initial,Tsc_goal,Tce_grasp,Tce_standoff_grasp,Tce_standoff_goal,T_mid)];
    
    % CSV Output
    csvwrite('refTrajectory.csv',refTrajectory);

% Defination

function TrajectryOut = refTrajectoryfun(Tse_initial,Tsc_goal,Tce_grasp,Tce_standoff_grasp,Tce_standoff_goal,T_mid)
        
    Method = 3;
        Trajectry=[];
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tse_initial,Tce_standoff_grasp,1,N,Method)) [0 0 0 0]']; % endeffector-initial to stand off grasp
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_standoff_grasp,Tce_grasp,1,N,Method)) [0 0 0 0]']; % stand off to grasp position
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_grasp,Tce_grasp,1,N,Method)) [1 0 0 0]']; % End effector Open to close
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_grasp,Tce_standoff_grasp,1,N,Method)) [1 0 0 0]']; % grasp to stand off grasp
        %Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_standoff_grasp,T_mid,1,N,Method)) [1 0 0 0]']; % stand off grasp to mid position
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_standoff_grasp,Tce_standoff_goal,1,N,Method)) [1 0 0 0]']; % to grasp stand off goal
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tce_standoff_goal,Tsc_goal,1,N,Method)) [1 0 0 0]']; % stand off goal to goal
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tsc_goal,Tsc_goal,1,N,Method)) [0 0 0 0]']; % close to open
        Trajectry = [Trajectry;cell2mat(ScrewTrajectory(Tsc_goal,Tce_standoff_goal,1,N,Method)) [0 0 0 0]']; % goal to stand off goal

        sequenceTraj=[];

        %arranging data
        
        for i =1:4:(size(Trajectry,1))
            for j=1:4:(size(Trajectry,2)-4)
                sequenceTraj = [sequenceTraj; [Trajectry(i,j:j+2) Trajectry(i+1,j:j+2) Trajectry(i+2,j:j+2) Trajectry(i:i+2,j+3)' Trajectry(i,size(Trajectry,2))]];
            end
        end
        TrajectryOut = sequenceTraj;
end
end