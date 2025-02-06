function [ucontrol,Xerr] = Feedback_Control(config_d,config_dnext,current_config,currentGlobal,dt,Je)

    % inputs 
    Tse = current_config;
    Tse_d = [config_d(1:3) config_d(10); config_d(4:6) config_d(11); config_d(7:9) config_d(12); 0 0 0 1]; % desired config
    Tse_dnext = [config_dnext(1:3) config_dnext(10); config_dnext(4:6) config_dnext(11); config_dnext(7:9) config_dnext(12); 0 0 0 1]; % desired at next step
     
    % gains Kpi =10,2 (smooth), 15.8 ,8 (Over)
    Kp = 10*eye(6); % Protptional gain
    Ki = 2*eye(6); % Integral gain
   
    Tff = TransInv(Tse_d)*Tse_dnext;
    Ted = TransInv(Tse)*Tse_d;

    Vff = se3ToVec(MatrixLog6(Tff)); % Feed-Forward Velocity in {d}
    adjV = Adjoint(Ted)*Vff; % Vff in {e}

    Xerr = se3ToVec(MatrixLog6(Ted)); % Twist error
    
    V = Vff + Kp*Xerr + Ki*Xerr*dt; % FF and PI Controller

    % To avoid joint Limits
    while(1)
        u = [pinv(Je)*V;config_d(13)];
        crossedj = [TestJoinLimits(u,currentGlobal,dt) velLimits(u,dt)];
        if isempty(crossedj)
            ucontrol = [pinv(Je,1e-4)*V;config_d(13)]; % {V} -> Control velocity
            break;
        else
            Je(:,crossedj(:)) = 0;
        end
    end

end