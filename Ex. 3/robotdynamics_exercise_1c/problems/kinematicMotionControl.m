function [ Dq ] = kinematicMotionControl(q, r_des, v_des)
% Inputs:
%  q          : current configuration of the robot
% r_des       : desired end effector position
% v_des       : desired end effector velocity
% Output: joint-space velocity command of the robot.

% Compute the updated joint velocities. This would be used for a velocity controllable robot
% TODO:

lambda=0;
kpp=0.5;

J = jointToPosJac_solution(q);
Jinv=pseudoInverseMat(J, lambda);

r=jointToPosition_solution(q);

dr=r_des - r;

Dq=Jinv*(v_des+kpp*dr);

end
