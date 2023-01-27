%% Calcolo coefficenti:
% M-------------------------------
for i = 1: length(q)
    M=M+(I_Jp_s{i})'*m{i}*I_Jp_s{i} + (I_Jr{i})'*R_Ik{i}*k_I_s{i}*R_Ik{i}'*I_Jr{i};
end

% b-------------------------------
for i = 1: length(q)
    b=b+I_Jp_s{i}' * m{i} * dAdt(I_Jp_s{i}, phi,dphi) * dphi + I_Jr{i}' * R_Ik{i} * k_I_s{i} * R_Ik{i}' * dAdt(I_Jr{i}, phi, dphi) * dphi + I_Jr{i}' * cross((I_Jr{i} * dphi),  R_Ik{i} * k_I_s{i} * R_Ik{i}' * I_Jr{i} * dphi);
end

% g-------------------------------
for i = 1: length(q)
    g = g - (I_Jp_s{i})' * m{i} * I_g_acc;
end

% Lambda-------------------------------
lambda = pinv(I_Je * pinv(M) * I_Je');

% mu-------------------------------
mu = lambda * I_Je * pinv(M) * b - lambda * I_dJ_e * dq;

% p-------------------------------
p = lambda * I_Je * pinv(M) * g;

%% pos_err, vel_err -------------------------------
pos_err = (I_r_ICd - I_r_IC);

w = I_Jp_C * dq;
vel_err = I_v_Cd - w;
%% force in task space to torque in joint space

tau_external_force = J_P_j' * F_j;
tau_external_force = J_R_j' * Tau_j;

%% forward dynamics ------------------------------- THIS
ddq = inv(M) * (tau - b - g);

%% joint impedance CONTROL (JOINT SPACE) (gravity compensation) THIS
tau = kp * (q_des - q) + kd * (q_dot_des - q_dot) + g;

%% gravity compensation plus weigth 
I_acc = (0; 9.81;0);
tau = g + I_J_weigth'*m_weigth*I_acc; 

%% inverse dynamic (JOINT SPACE) ------------------------------- NOPE
% 
q_double_dot = kp * (q_des - q) + kd * (q_dot_des - q_dot) +q_dot_dot_des;
y = q_double_dot;
tau = M * y + b + g;
     
%% inverse dynamic CONTROL (TASK SPACE) ------------------------------- THIS
% 
q_double_dot = pinv(I_Je_C)* (w_dot_ref + kd * vel_err + kp * pos_err - I_dJ_e * q_dot);
y = q_double_dot;
tau = M * y + b + g;

%% inverse dynamic CONTROL (TASK SPACE) ------------------------------- hybrid senza forze
w_dot = (w_dot_ref + kd * vel_err + kp * pos_err);
tau = I_Je' * (lambda *w_dot + mu + p);

%% operational space CONTROL HYBRID------------------------------- THIS
w_dot_e_star = kp * pos_err + kd * vel_err + w_dot_ref;
tau = I_Je' * (lambda * Sm * w_dot_e_star + Sf * I_F_E + mu + p);

%% energies
for i = 1:length(phi)
    rsi = T_Ik{i};
    r = rsi(1:3,4)+R_Ik{i} * k_r_ks{i};
    F = m{i} * I_g_acc;
    enPot = enPot - (r)' *  F;
end

enKin = 0.5 * dphi' * M * dphi;
hamiltonian = enKin + enPot;