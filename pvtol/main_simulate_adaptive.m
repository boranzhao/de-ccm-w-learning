%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of planning and control of a planar quadrotor with adaptive CCM 
% proposed in 
%  B. Lopez and J.J. Slotine. Contraction Metrics in Adaptive Nonlinear
%  Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;close all;
addpath('../ccm/control_law_online/');
addpath('../ccm/metric_search_offline/');
addpath('../utilities/');
%%
% ---------------------- load plant and controller ------------------------
file_controller = 'ccm_0.8_plim_0.33pi.mat';  
load(file_controller);

% start and end positions
x0xF_config = 1; % {1,2,3}
dist_config.center = [5,5]';
dist_config.radius = 5;


plant.phi = @(x) [x(4)^2/(1+(x(1)-5)^2+(x(2)-5)^2) x(5)^2/(1+(x(1)-5)^2+(x(2)-5)^2); 
                  x(4)^2/(1+(x(1)-5)^2+(x(2)-5)^2) x(5)^2/(1+(x(1)-5)^2+(x(2)-5)^2)];

% dist learned
params = load('params_better.mat').params;
prd_dist = @(x) -uncer_func_better(x,params);
dist_config.distLearned = @(x) learned_dist_fcn(x,prd_dist);
%% adaptive control setting
controller.adaptive_comp = 1;       %{0,1} whether to add adaptive_comp 
controller.adaptation_gain = 100*diag([1 1]);

% --------------------- actual disturbance settings -----------------------
% dist_config.dist_fcn = @(x) (x(4)^2+x(5)^2)*[0.1;0.1];
% dist_config.dist_fcn = @(t,x) actual_dist_fcn(t,x,dist_config.center,dist_config.radius);
dist_config.dist_fcn = @(t,x) actual_dist_fcn(x,dist_config.center,dist_config.radius);


% --------(learned) disturbance model, to be replaced by a NN model--------
use_distModel_in_planning_control = 0;  % {1,0}: whether to include a (learned) disturbance model in planning and control
% distLearned = @(x) learned_dist_fcn(x,dist_config.center,dist_config.radius);
% distLearned = @(x) zero_dist(x);        % Zero disturbance model
controller.use_distModel_in_planning_control = use_distModel_in_planning_control;

%  -----------------------simulation settings -----------------------------
sim_config.replan_nom_traj = 0;     % {1,0}: whether to replan a trajectory
sim_config.include_obs = 1;         % {1,0}: whether to include the obstacles
sim_config.include_dist = 1;        % {1,0}: whether to include the disturbance  
sim_config.save_sim_rst = 1;        % {1,0}: whether to save simulation results
sim_config.tight_input_bnd = 1;     % {1,0}: whether to tighten the input bnd for trajectory generation
sim_config.include_tube = 1;        % {1,0}: whether to include a safety tube when plannign the trajectories
sim_config.step_size = 0.0002;      % step size for simulation with ode1 solver (not used when using a variable-step solver)

use_generated_code = 1;             % whether to use the generated codes for simulations: using generated codes can accelerate by at least one fold

n = 6; nu = 2;
if x0xF_config == 1
        x0 = [2;0;zeros(4,1)];     % initial state
        xF = [10 10 0 0 0 0]';      % final state
elseif x0xF_config == 2
        x0 = [8;0;zeros(4,1)];     % initial state
        xF = [2 10 0 0 0 0]';      % final state
elseif x0xF_config == 3
        x0 = [0;6;zeros(4,1)];     % initial state
        xF = [10 6 0 0 0 0]';      % final state
end

% x0 = [0;0;zeros(4,1)];                    % initial state
% xF = [10 10 0 0 0 0]';              % final state
umax = 1.5*plant.m*plant.g;           % control limit
% ----- bounds for input and states for using OptimTraj to plan trajs.-----
u_bnd = [0 0; umax umax]';
x_bnd = [-inf -inf -state_set.p_lim -state_set.vx_lim, -state_set.vz_lim, -state_set.pd_lim;
          inf  inf  state_set.p_lim  state_set.vx_lim   state_set.vz_lim,   state_set.pd_lim]';
tube_u = 1; tube_xz = 0.2;
if sim_config.tight_input_bnd == 1
    u_bnd = u_bnd +[0 0; -tube_u -tube_u]';    
end

%% Plan or load a nominal trajecotory 
plan_load_nom_traj;

%% Change the starting point and simulation duration for simulation
if x0xF_config == 1
    x0 = [0;0;zeros(4,1)];     % initial state
elseif x0xF_config == 2
    x0 = [10;0;zeros(4,1)];     % initial state
elseif x0xF_config == 3
    x0 = [0;4;zeros(4,1)];     % initial state
end

%% Formulate the NLP problem for geodesic computation
controller.use_generated_code = use_generated_code;
lambda = controller.lambda;
%  problem setting for geodesic computation
D = 2;      % degree of the polynomial
N_steps = D+6;    % stopping index for the CGL (Chebyshev-Gauss-Lobatto) nodes: #notes N+1

% optimization variables: chebyshev coefficients for geodesics
% [c_10, c_11,..., c_1D, ..., c_n0, c_n1,..., c_nD]

% obtain chebyshev pseudospectral numerics (to be used for computing the
% integral)
[s,w_cheby] = clencurt(N_steps); % t is 1 by N+1, with values lying b/t 0 and 1.
% evaluate the value of the CCM
W = zeros(n,n,N_steps+1);

% compute Cheby basis at all points
[T, Tdot] = compute_cheby(N_steps,D,s); % Both T and T_dot are D+1 by N+1
% for equality constraints
Aeq = [kron(eye(n),T(:,1)'); kron(eye(n),ones(1,D+1))];
Aeq = sparse(Aeq);

% --------- formulate and solve the NLP problem using OPTI --------------
ndec = n*(D+1);
if controller.use_generated_code == 1
    costf = @(c) RiemannEnergy1_mex(c,n,D,N_steps,T,Tdot,w_cheby);
    grad = @(c) energyGradient1_mex(c,n,D,N_steps,T,Tdot,w_cheby);     
else    
    costf = @(c) RiemannEnergy(c,n,D,N_steps,T,Tdot,w_cheby,controller.W_fcn);
    grad = @(c) energyGradient(c,n,D,N_steps,T,Tdot,w_cheby,controller.W_fcn,controller.dW_dxi_fcn); 
end

geodesic.D = D; geodesic.N = N_steps; geodesic.ndec = ndec;
geodesic.T = T; geodesic.Tdot = Tdot;
geodesic.Aeq = Aeq; 
geodesic.costf = costf;
geodesic.grad = grad;
geodesic.w_cheby = w_cheby;

beq = zeros(2*n,1);c0 = zeros(n*(D+1),1);
% add some bounds to mitigate numerical issues when computing the geodesic
lb = -20*ones(n,D+1);
ub = 20*ones(n,D+1);
lb(3:4,:) = -5*ones(2,D+1);  % phi and vx
ub(3:4,:) = 5*ones(2,D+1);   % phi and vx
lb = lb';lb = lb(:);
ub = ub';ub= ub(:);

% --------------- re-generate the code is necessary after change of ------
% controller or geodesic optimization settings: remember to re-generate the
% m-file functions, e.g., dW_dphi, dW_dvx, etc., first.------------------  
if controller.use_generated_code 
    answer = questdlg('Are the generated codes for this particular scenario?','Question for using C-code in simulation','Yes','No','No');
    switch answer 
        case 'Yes'
        case 'No'
            error('You cannot continue without including the generated codes for this specific scenario!');
    end
end

%     [copt1,Erem,exitflag,info] = solve(Opt,c0);

% ---------- for using ipopt solver ---------------------------------------
% opts_opti = optiset('solver','ipopt','maxiter', 500,'display','iter');  %,,'derivCheck','on'
% Opt = opti('fun',geodesic.costf,'grad',geodesic.grad,'eq',geodesic.Aeq,beq,...
%             'bounds',lb,ub,'ndec',geodesic.ndec,'x0',c0,'options',geodesic.opts_opti);
% geodesic.nlprob = convIpopt(Opt.prob,geodesic.opts_opti); 
% -------------------------------------------------------------------------

% ----------for using matlab fmincon solver--------------------------------
opts_opti = optiset('solver','matlab','maxiter',500,'tolrfun',5e-6,'tolafun',5e-6,'display','off','derivCheck','off'); 
Opt = opti('fun',costf,'grad',grad,'eq',Aeq,beq,'bounds',lb,ub,'ndec',ndec,'x0',c0,'options',opts_opti);
geodesic.nlprob = convMatlab(Opt.prob,opts_opti); 
geodesic.nlprob.options = ...
optimoptions(@fmincon,'Display','off','HessianApproximation','lbfgs',...
'MaxIterations',opts_opti.maxiter,'SpecifyObjectiveGradient',true,'CheckGradients',false,...
'OptimalityTolerance',opts_opti.tolrfun,'FunctionTolerance',opts_opti.tolrfun,'FunValCheck','on','StepTolerance',1.0e-8);
% -------------------------------------------------------------------------

geodesic.opts_opti = opts_opti;
controller.x_nom_fcn = x_nom_fcn;
controller.u_nom_fcn = u_nom_fcn;
controller.geodesic = geodesic;
controller.w_nom = 0;  % nominal value for disturbances

% simulate
dist0 = norm(x0); 
thetahat0 = zeros([2,1]);
% -------------------------------------------------------------------------

% --------for additionally outputing control inputs and Reim. energy-------
% compute the initial Riemann energy function value
[ue0,thetahat_dot0] = adaptive_ccm_law(0,x0,plant,controller,thetahat0,dist_config);
tic;

%% manual simulation
% OPTIONS = odeset('RelTol',2e-3,'AbsTol',1e-5);
% tVec = 0:sim_config.step_size:duration;
% N_steps = length(tVec);
% xTraj = zeros(n,N_steps);
% uTraj = zeros(nu,N_steps);
% energyTraj = zeros(1,N_steps);
% thetahatTraj = zeros(2,N_steps);
% 
% x = x0;
% x_thetahat = [x0;thetahat0]; 
% 
% u = ue0(1:2); thetahat_dot = thetahat_dot0;
% uTraj(:,1) = u;
% enertyTraj(:,1) = ue0(end);
% xTraj(:,1)=x0;
% thetahatTraj(:,1) = thetahat_dot0;
% for i=1:N_steps-1
%     t = tVec(i);
%     n = plant.n;
%     
%     [d_t,d_state] = ode45(@(t,state) pvtol_dyn2(t,state,u,thetahat_dot,plant,sim_config,dist_config),[tVec(i) tVec(i+1)],x_thetahat,OPTIONS); %,ode_opts)
%     x_thetahat = d_state(end,:)';
%     x = x_thetahat(1:n); xTraj(:,i+1)= x;
%     thetahat = x_thetahat(n+1:n+2); thetahatTraj(:,i+1) = thetahat;    
%     
%     [ue,thetahat_dot]= adaptive_ccm_law(t,x,plant,controller,thetahat);
%     u = ue(1:end-1); uTraj(:,i+1) = u;
%     energyTraj(1,i+1) = ue(end);
% end

%% Based on ODE solver
x_u_e_thetahat0 = [x0;controller.u_nom_fcn(0);ue0(end);zeros([2,1])]; % state, input, energy,estimated paras; (,estimated disturbance);
Klp = [500*ones(3,1)];  
%ode23 is fastest, followed by ode45 
% OPTIONS = odeset('RelTol',2e-3,'AbsTol',1e-5);
% OPTIONS = odeset('RelTol',2e-5,'AbsTol',1e-6);
% duration = 10;
% [tVec,x_u_e_thetahat_Traj] = ode23(@(t,state) pvtol_dyn(t,state,Klp,plant,controller,sim_config,dist_config),[0 duration],x_u_e_thetahat0,OPTIONS); %,ode_opts)
% 
% ----------------------- ode1: fixed step ----------------------------
duration = 5.3;
times = 0:sim_config.step_size:duration;
x_u_e_thetahat_Traj = ode1(@(t,state) pvtol_dyn(t,state,Klp,plant,controller,sim_config,dist_config,duration),times,x_u_e_thetahat0); %,ode_opts)
% ---------------------------------------------------------------------
x_u_e_thetahat_Traj = x_u_e_thetahat_Traj';
xTraj = x_u_e_thetahat_Traj(1:n,:);
uTraj = x_u_e_thetahat_Traj(n+1:n+2,:);
energyTraj = x_u_e_thetahat_Traj(n+3,:);            % Riem. Energy
thetahatTraj = x_u_e_thetahat_Traj(n+4:n+5,:);      

toc;
%% plot the result
% load("backup2_adaptiveCCM_ode1_w_learning_03_00_88_gain10.mat")
plot_and_save

%% some functions
function dstate = pvtol_dyn(t,x_u_e_thetahat,Klp,plant,controller,sim_config,dist_config,duration)
persistent uTrajin
if isempty(uTrajin)
    uTrajin = zeros(2,ceil(duration/sim_config.step_size));
end
delaysteps = 50;
n = plant.n;
x = x_u_e_thetahat(1:n);
u_e = x_u_e_thetahat(n+1:n+3);
thetahat = x_u_e_thetahat(n+4:n+5);

[ue,thetahat_dot]= adaptive_ccm_law(t,x,plant,controller,thetahat,dist_config);

% u = ue(1:end-1); 
uTrajin(:,round(t/sim_config.step_size+1)) = ue(1:2);
wt = dist_config.dist_fcn(t,x);

% update the states of actual system, state predictor, ...
if round(t/sim_config.step_size) < delaysteps
    dstate = [plant.f_fcn(x); -Klp.*u_e;thetahat_dot]+[plant.B_fcn(x)*[0;0]; Klp.*ue;zeros([2,1])];
else
    dstate = [plant.f_fcn(x); -Klp.*u_e;thetahat_dot]+[plant.B_fcn(x)*uTrajin(:,1+max(0,round(t/sim_config.step_size-delaysteps))); Klp.*ue;zeros([2,1])];
end
if sim_config.include_dist == 1
   dstate(1:n,:) = dstate(1:n,:) + plant.B_fcn(x)*wt;
end
end

function [intensity,distance_to_center] = dist_distribution(X,Z,center,radius)
distance_to_center = sqrt((X-center(1)).^2 + (Z-center(2)).^2);
intensity = 1./(distance_to_center.^2+1);

% --------------- using an inverse function 2 ----------------
% intensity = 1./(sqrt(distance_to_center.^2)+1);
end

% ---------------- True disturbance --------------------
% function dist_force = actual_dist_fcn(t,x,center,radius)
% dist_force = [x(4)^2*(-1+0.3*sin(2*t)) x(5)^2*(-1+0.3*sin(2*t)); x(4)^2*(-1+0.3*cos(2*t)) x(5)^2*(-1+0.3*cos(2*t))]*[0.15;0.15]; 
% end
function dist_force = actual_dist_fcn(x,center,radius)
max_damping = 0.5;
% max_damping = 0;

[dist_intensity,~] = dist_distribution(x(1,:),x(2,:),center,radius);
% dist_intensity = 0.3;
dist_force_max = (x(4,:)^2+x(5,:)^2)*max_damping;
dist_force = [-1; -1]*(dist_intensity.*dist_force_max); 
end

%-------------learned disturbance model--------------------
function [dist_force, d_force_dx]= learned_dist_fcn(x,prd_dist)
[n,N]  = size(x);

d_force_dx = zeros(2,n,N);
% dist_force = zeros(2,N);

x_reduced = x([1 2 4 5],:)';
dist_force = prd_dist(x_reduced);
if nargout>1
    [n,N] = size(x);
%     tic;
%     for i = 1:N        
%         % ---------------- using an inverse function -----------------
% %         xx = x(:,i);
%         xx_reduced = x_reduced(i,:);
% 
% %       tic;
%         jac = jacobian(prd_dist,xx_reduced);
% %         toc;
%         d_force_dx(:,[1 2 4 5],i) = jac;
%     end
%     toc;

    jac = jacobianN(prd_dist,x_reduced);
    d_force_dx(:,[1 2 4 5],:) = jac;
end

end

function jac = jacobianN(prd_dist,x)
% only works for 2x4 jacobian
% x is Nx4
N = size(x,1);
jac = zeros(2*N,4);
e = 1e-5; evec = e*ones(N,1);
x_pts = zeros(2*N,4);
x_pts(1:2:end,:) = x;
x_pts(2:2:end,:) = x;


x_pt1 = x_pts; 
x_pt1(1:2:end,1) = x_pt1(1:2:end,1)+ evec;
x_pt1(2:2:end,1) = x_pt1(2:2:end,1)- evec;

x_pt2 = x_pts; 
x_pt2(1:2:end,2) = x_pt2(1:2:end,2)+ evec;
x_pt2(2:2:end,2) = x_pt2(2:2:end,2)- evec;

x_pt3 = x_pts; 
x_pt3(1:2:end,3) = x_pt3(1:2:end,3)+ evec;
x_pt3(2:2:end,3) = x_pt3(2:2:end,3)- evec;

x_pt4 = x_pts; 
x_pt4(1:2:end,4) = x_pt4(1:2:end,4)+ evec;
x_pt4(2:2:end,4) = x_pt4(2:2:end,4)- evec;

temp = (prd_dist(x_pt1(1:2:end,:))-prd_dist(x_pt1(2:2:end,:)))/(2*e);
jac(:,1) = temp(:);    
temp = (prd_dist(x_pt2(1:2:end,:))-prd_dist(x_pt2(2:2:end,:)))/(2*e);
jac(:,2) = temp(:);
temp = (prd_dist(x_pt3(1:2:end,:))-prd_dist(x_pt3(2:2:end,:)))/(2*e);
jac(:,3) = temp(:);
temp = (prd_dist(x_pt4(1:2:end,:))-prd_dist(x_pt4(2:2:end,:)))/(2*e);
jac(:,4) = temp(:); 

jac = permute(reshape(jac,2,N,4),[1,3,2]);
end

function [dist_force, d_force_dx]= zero_dist(x)
[n,N]  = size(x);
dist_force = zeros(2,N); 
if nargout>1
    d_force_dx = zeros(2,n,N);    
end
end