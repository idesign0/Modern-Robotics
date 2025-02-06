function vellimitscrossed = velLimits(controlVec,dt)

% Velocity Limits

crossed = [];
joints = controlVec*dt;

if abs(joints(5)) > 0.1
      crossed = [crossed,6];
end
if abs(joints(6)) > 0.1
      crossed = [crossed,6];
end
if abs(joints(7)) > 0.1
     crossed = [crossed,7];
end
if abs(joints(8)) > 0.1
     crossed = [crossed,8];
end
% if abs(joints(9)) > 0.1
%      crossed = [crossed,9];
% end

vellimitscrossed = crossed;
end