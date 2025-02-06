function newConfig = Nextstep(control,config,dt)
    
    % Inputs
    intConfig = config;
    controlVec = control';
    
    % NextState
    outputTrace=[];
    newConfig = [outputTrace;States(intConfig,controlVec,dt)];
    
    % definations nextState
    function currentPos = States(intConfig,controlVec,dt)
            actualCC = intConfig(1:3);
            actwhlC = intConfig(9:12);
            actarm = intConfig(4:8);
       
            actualCC = (actualCC' + F_phimat(actualCC(1))*((controlVec(1:4)'*dt)))'; % du -> Vbody -> Vbase (global) {phi, X, Y}
            actwhlC = actwhlC + controlVec(1:4)*dt; % {W1,W2,W3,W4}
            actarm = actarm + controlVec(5:9)*dt; % {Thta1 : Thta5}

            gripperstate = controlVec(10);
            
            currentPos = [actualCC actarm actwhlC gripperstate];
    end
    
end