function limitscrossed = TestJoinLimits(controlVec,cG,dt)

%joint max value function

crossed = [];
joints = controlVec(5:9)*dt + cG(4:8)';

if (joints(1)) < -1 || (joints(1)) > 1
     crossed = [crossed,1];
end
if (joints(2)) < -1.117 || (joints(2)) > 0
     crossed = [crossed,2];
end
if (joints(3)) < -2 || (joints(3)) > 0.5
     crossed = [crossed,3];
end
if (joints(4)) < -2 || (joints(4)) > 0.5
    crossed = [crossed,4];
end

limitscrossed = crossed+4;
end