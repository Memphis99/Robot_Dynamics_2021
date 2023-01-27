% generate equations of motion
function eom = generate_eom_solution(gen_cor, kin, dyn, jac)
% By calling:
%   eom = generate_eom(gen_cor, kin, dyn, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion. These are additionally
% converted to matlab scripts.

%% Setup
phi = gen_cor.phi;      % Generalized coordinates (6x1 sym)
dphi = gen_cor.dphi;    % Generalized velocities (6x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (6x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (6x1 cell)->(3x3 sym)

k_I_s = dyn.k_I_s;      % Inertia tensor of body k in frame k (6x1 cell)->(3x3 sym)
m = dyn.m;              % Mass of body k (6x1 cell)->(1x1 double)
I_g_acc = dyn.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = dyn.k_r_ks;    % CoM location of body k in frame k (6x1 cell)->(3x1 double)

I_Jp_s = jac.I_Jp_s;    % CoM Positional Jacobian in frame I (6x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr;        % CoM Rotational Jacobian in frame I (6x1 cell)->(3x6 sym)

eom.M = sym(zeros(6,6));
eom.g = sym(zeros(6,1));
eom.b = sym(zeros(6,1));
eom.hamiltonian = sym(zeros(1,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(6,6));
for k = 1:length(phi)
    M = M + I_Jp_s{k}'*m{k}*I_Jp_s{k} + I_Jr{k}'*R_Ik{k}*k_I_s{k}*R_Ik{k}'*I_Jr{k};
end

%% Compute gravity terms
% fprintf('Computing gravity vector g... ');
% enPot = sym(0);
% for k=1:length(phi)
%     enPot = enPot - m{k}*I_g_acc'*[eye(3) zeros(3,1)]*T_Ik{k}*[k_r_ks{k};1];
% end
% g = jacobian(enPot, phi)';


g = sym(zeros(6,1));
for k=1:length(phi)
   g = g - I_Jp_s{k}'* m{k}* I_g_acc;
end


%% Compute nonlinear terms vector
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(6,1));

for i=1:size(phi)
    b=b + I_Jp_s{i}.'*m{i}*dAdt(I_Jp_s{i}, phi, dphi)*dphi +...
      (I_Jr{i}).'*R_Ik{i}*k_I_s{i}*(R_Ik{i}).'*dAdt(I_Jr{i}, phi, dphi)*dphi +...
      (I_Jr{i}).'*cross(I_Jr{i}*dphi, R_Ik{i}*k_I_s{i}*(R_Ik{i}).'*I_Jr{i}*dphi);
end
fprintf('done!\n');


%% Compute energy
fprintf('Computing total energy... ');

enKin = 0.5*dphi'*M*dphi;

enPot = sym(zeros(1,1));
%get r_ks to the inertial frame I with the tranf. matrix
for i=1:size(phi)
    enPot = enPot - (T_Ik{i}(1:3, 1:4)*[k_r_ks{i}; 1]).'*m{i}*I_g_acc;
end

hamiltonian = enKin + enPot;

fprintf('done!\n');


%% Generate matlab functions

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating eom scripts... ');
fprintf('M... ');
matlabFunction(M, 'vars', {phi}, 'file', strcat(dpath,'/M_fun_solution'), 'Optimize', true);
fprintf('g... ');
matlabFunction(g, 'vars', {phi}, 'file', strcat(dpath,'/g_fun_solution'), 'Optimize', true);
fprintf('b... ');
matlabFunction(b, 'vars', {phi, dphi}, 'file', strcat(dpath,'/b_fun_solution'), 'Optimize', true);
fprintf('hamiltonian... ');
matlabFunction(hamiltonian, 'vars', {phi, dphi}, 'file', strcat(dpath,'/hamiltonian_fun_solution'), 'Optimize', true);
fprintf('done!\n');


%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
eom.hamiltonian = hamiltonian;
eom.enPot = enPot;
eom.enKin = enKin;

end
