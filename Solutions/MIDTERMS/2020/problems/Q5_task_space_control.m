function [tau] = Q5_task_space_control( params, gc, I_r_ICd, I_v_Cd, I_a_Cd)
% Task-space inverse dynamics controller tracking a desired end-effector motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - I_r_ICd   : the desired position (3x1) of the camera w.r.t. the inertial frame expressed in the inertial frame.
%   - I_v_Cd    : the desired linear velocity (3x1) of the camera in the inertial frame.
%   - I_a_Cd    : the desired linear acceleration (3x1) of the camera in the inertial frame.
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_Q5; % P gain matrix for camera position (3x3 diagonal matrix)
kd = params.kd_Q5; % D gain matrix for camera velocity  (3x3 diagonal matrix)

% Find jacobians, positions and orientation based on the current
I_Jp_C = I_Jp_C_fun(q); % Positional Jacobian of end effector
I_Jr_C = I_Jr_C_fun(q); % Rotational Jacobian of end effector
I_dJp_C = I_dJp_C_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_dJr_C = I_dJr_C_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)

% Geometrical Jacobian
I_J_C = [I_Jp_C; I_Jr_C];
I_dJ_C = [I_dJp_C; I_dJr_C];

T_IC = T_IC_fun(q); % Homogeneous transformation from frame C to frame I
I_r_IC = T_IC(1:3, 4);

%% Compute torque
tau = zeros(3,1); % TODO

j_invm = I_Jp_C/M; 
lambda = pseudoInverseMat(j_invm*I_Jp_C', 0.001); % Inertia matrix in the operational space
mu = lambda*(j_invm*b - I_dJp_C*dq); % Nonlinear term in the operational space
p =  lambda*j_invm*g;  % Gravity term in the operational space

pos=I_r_IC;
vel=I_Jp_C*dq;
acc=I_a_Cd + kp*(I_r_ICd - pos) + kd*(I_v_Cd - vel);

I_F_C_des = lambda * acc + mu + p;

tau = I_Jp_C'*I_F_C_des;
end
