%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulations in the presence of time delay and no learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;close all;
controller_type = 'ccm';            % {'ccm','ccm-dnn','ccm-dnn-de'} 'de' refers to disturbance estimation
% ---------------------- load plant and controller ------------------------
file_controller = 'ccm_0.8_plim_0.33pi.mat';          
load(file_controller);
% start and end positions
x0xF_config = 1; % {1,2,3,4,5,6}
delay = 0.01; % second. 

% ---whether to include dist. estimation and error bound in CCM control----
%{0,1,2}: 
%0 for ignoring, 
%1 for estimating the remainder disturbance $\tilde ur$ (between the learned disturbance and true disturbance), 
%2 for estimating the total disturbance d
%3 for estimating the total disturbance d but filter only the reminder
%uncertainty estimate d-d_hat
controller.distEstScheme = 3;       
controller.use_distEst_errBnd = 1; 
controller.filter_distEst = 1;      %{0,1}, whether to filter the estimated disturbance to remove the high gain components

% --------------------- actual disturbance settings -----------------------
dist_config.center = [5,5]';
dist_config.radius = 5;
dist_config.dist_fcn = @(t,x) actual_dist_fcn(t,x,dist_config.center,dist_config.radius);

% --------(learned) disturbance model, to be replaced by a NN model--------
use_distModel_in_planning_control = 1;  % {1,0}: whether to include a (learned) disturbance model in planning and control
% distLearned = @(x) learned_dist_fcn(x,dist_config.center,dist_config.radius);
% distLearned = @(x) zero_dist(x);        % Zero disturbance model
controller.use_distModel_in_planning_control = use_distModel_in_planning_control;


% Ziyao
params = load('params_perfect.mat').params;
prd_dist = @(x) -uncer_func_perfect(x,params);
distLearned = @(x) learned_dist_fcn(x,prd_dist);
% distLearned = @(x) perfect_learner(x,dist_config.center,dist_config.radius);
       
% ------------------- disturbance estimation setting ----------------------
distEst_config.a_pred = 10;                    % positive constant used in the state predictor
distEst_config.Ts = 0.002;                    % estimation sampling time
distEst_config.adapt_gain = -distEst_config.a_pred/(exp(distEst_config.a_pred*distEst_config.Ts)-1);

%% computating the estimation error bound
% compute_est_err_bnd; 
% 
% set to an artificial value
distEst_config.est_errBnd = 0.2;
% distEst_config.est_errBnd = 2.5*sqrt(2);

%  -----------------------simulation settings -----------------------------
sim_config.replan_nom_traj = 0;     % {1,0}: whether to replan a trajectory
sim_config.include_obs = 1;         % {1,0}: whether to include the obstacles
sim_config.include_dist = 1;        % {1,0}: whether to include the disturbance  
sim_config.save_sim_rst = 1;        % {1,0}: whether to save simulation results
sim_config.tight_input_bnd = 0; 
% {1,0}: whether to tighten the input bnd for trajectory generation
sim_config.include_tube = 1;        % {1,0}: whether to include a safety tube when plannign the trajectories
sim_config.step_size = 1e-3;      % step size for simulation with ode1 solver (not used when using a variable-step solver)
sim_config.delaysteps = delay/sim_config.step_size;
use_generated_code = 1;             % whether to use the generated codes for simulations: using generated codes can accelerate by at least one fold

n = 6; nu = 2;
if x0xF_config == 1
        x0 = [2;0;zeros(4,1)];     % initial state
        xF = [8 10 0 0 0 0]';      % final state
elseif x0xF_config == 2
%         x0 = [8;0;zeros(4,1)];     % initial state
%         xF = [2 10 0 0 0 0]';      % final state
        xF = [8;0;zeros(4,1)];     % initial state
        x0 = [2 10 0 0 0 0]';      % final state
elseif x0xF_config == 3
        x0 = [0;6;zeros(4,1)];     % initial state
        xF = [10 6 0 0 0 0]';      % final state
% elseif x0xF_config == 3
%         x0 = [0;2;zeros(4,1)];     % initial state
%         xF = [10 6 0 0 0 0]';      % final state
elseif x0xF_config == 4
        x0 = [0;6;zeros(4,1)];     % initial state
        xF = [10 2 0 0 0 0]';      % final state
elseif x0xF_config == 5
        x0 = [2;10;zeros(4,1)];     % initial state
        xF = [8 0 0 0 0 0]';      % final state
elseif x0xF_config == 6
        x0 = [2;0;zeros(4,1)];     % initial state
        xF = [8 10 0 0 0 0]';      % final state
elseif x0xF_config == 7
        x0 = [0;7;zeros(4,1)];     % initial state
        xF = [6 0 0 0 0 0]';      % final state
elseif x0xF_config == 8
        x0 = [4;0;zeros(4,1)];     % initial state
        xF = [10 7 0 0 0 0]';      % final state
elseif x0xF_config == 9
        x0 = [0;8;zeros(4,1)];     % initial state
        xF = [10 8 0 0 0 0]';      % final state
end

% x0 = [0;0;zeros(4,1)];                    % initial state
% xF = [10 10 0 0 0 0]';              % final state
duration = 13;                      % (estimated) time  13
umax = 1.5*plant.m*plant.g;           % control limit
% ----- bounds for input and states for using OptimTraj to plan trajs.-----
u_bnd = [0 0; umax umax]';
x_bnd = [-inf -inf -state_set.p_lim -state_set.vx_lim, -state_set.vz_lim, -state_set.pd_lim;
          inf  inf  state_set.p_lim  state_set.vx_lim   state_set.vz_lim,   state_set.pd_lim]';
tube_u = 1; tube_xz = 0.2;
if sim_config.tight_input_bnd == 1
    u_bnd = u_bnd +[0 0; -tube_u -tube_u]';    
end
% return
%% Plan or load a nominal trajecotory 
plan_load_traj;

%% Change the starting point and simulation duration for simulation
if x0xF_config == 1
        x0 = [0;0;zeros(4,1)];     % initial state
elseif x0xF_config == 2
        x0 = [0;10;zeros(4,1)];     % initial state
% elseif x0xF_config == 3
%         x0 = [0;2;zeros(4,1)];     % initial state
elseif x0xF_config == 3
        x0 = [0;4;zeros(4,1)];     % initial state
elseif x0xF_config == 4
        x0 = [0;6;zeros(4,1)];     % initial state
elseif x0xF_config == 5
        x0 = [2;10;zeros(4,1)];     % initial state
elseif x0xF_config == 6
        x0 = [2;0;zeros(4,1)];     % initial state
elseif x0xF_config == 7
        x0 = [0;7;zeros(4,1)];     % initial state
elseif x0xF_config == 8
        x0 = [4;0;zeros(4,1)];     % initial state
elseif x0xF_config == 9
        x0 = [0;8;zeros(4,1)];     % initial state
end
% return

%% Formulate the NLP problem for geodesic computation
set_geodesic_prob;

controller.w_nom = 0;  % nominal value for disturbances

% simulate
dist0 = norm(x0); 
% -------------------------------------------------------------------------
% [times,xTraj] = ode23s(@(t,x) plant.f_fcn(x)+plant.B*ccm_law(t,x,plant,controller),[0 duration],x0); 
% toc;
% return;
% ode_opts = odeset('MaxStep',5e-1);
% --------for additionally outputing control inputs and Reim. energy-------
% compute the initial Riemann energy function value
ue = ccm_law(0,x0,plant,controller,distLearned,[0 0]',distEst_config.est_errBnd);
% ue = robust_ccm_law(0,x0,plant,controller,distLearned,[0 0]',distEst_config.est_errBnd);
x_xhat_u_d_0 = [x0;x0;controller.u_nom_fcn(0);ue(end);[0;0];[0;0]]; % state,state, input, energy, true disturance,estimated disturbance;
Klp = [500*ones(5,1);30;30]; % the last two is for filtering estimated uncertainties. 
tic;

% ---------------- ode23 is fastest, followed by ode45 ------------------
% OPTIONS = odeset('RelTol',2e-3,'AbsTol',1e-5);
% [times,x_xhat_u_d_Traj] = ode23(@(t,xu) pvtol_dyn(t,xu,Klp,plant,controller,sim_config,dist_config,distLearned,distEst_config),[0 duration],x_xhat_u_d_0,OPTIONS); %,ode_opts)
% -----------------------------------------------------------------------

% ----------------------- ode1: fixed step ----------------------------

times = 0:sim_config.step_size:duration;
x_xhat_u_d_Traj = ode1(@(t,xu) pvtol_dyn(t,xu,Klp,plant,controller,sim_config,dist_config,distLearned,distEst_config,duration),times,x_xhat_u_d_0); %,ode_opts)
% ---------------------------------------------------------------------

toc;
x_xhat_u_d_Traj = x_xhat_u_d_Traj';
xTraj = x_xhat_u_d_Traj(1:n,:);
xhatTraj = x_xhat_u_d_Traj(n+1:2*n,:);
uTraj = x_xhat_u_d_Traj(2*n+1:2*n+2,:);
energyTraj = x_xhat_u_d_Traj(2*n+3,:);        % Riem. Energy
distTraj = x_xhat_u_d_Traj(2*n+4:2*n+5,:);      % True disturbance
estDistTraj = x_xhat_u_d_Traj(2*n+6:2*n+7,:);      % Estimated disturbance

%% plot the result
plot_rst;
distLearnedTraj = distLearned(xTraj);
figure(2);
clf;hold on;
if sim_config.include_dist == 1
    if controller.distEstScheme == 1 || controller.distEstScheme == 3
        plot(times,distTraj(1,:),'k',times,estDistTraj(1,:)+distLearnedTraj(1,:),'r--'); 
        plot(times,distTraj(2,:),'k--',times,estDistTraj(2,:)+distLearnedTraj(2,:),'r-.');
    else
        plot(times,distTraj(1,:),'k',times,estDistTraj(1,:),'r--'); 
        plot(times,distTraj(2,:),'k--',times,estDistTraj(2,:),'r-.');
    end
    ylabel('Disturbance')
    xlabel('Time (s)');
    legend('Actual: $\tilde{d}_1$','Estimated: $\hat{\sigma}_1$','Actual: $\tilde{d}_2$','Estimated: $\hat{\sigma}_2$','Interpreter','Latex');    
end


%%
if sim_config.save_sim_rst == 1
    file_name = ['ACC/DECCM_perfect_learning_' num2str(sim_config.step_size*1000) 'ms_' num2str(delay) 's_delay'];
    file_name = [file_name '_' num2str(x0(1)) num2str(x0(2)) '_' num2str(xF(1)) num2str(xF(2)) '.mat'];
    save(file_name,'times','xTraj','uTraj','xnomTraj','unomTraj','energyTraj','dist_config','sim_config','plant','controller','estDistTraj');
end


%% some functions
function dxdt = pvtol_dyn(t,x_xhat_u_d,Klp,plant,controller,sim_config,dist_config,distLearned,distEst_config,duration)
persistent distEst Bsigmahat uTrajin
if isempty(distEst)
    distEst = [0;0];
    Bsigmahat = zeros(6,1);
%     uehold = [0;0];
    uTrajin = zeros(2,ceil(duration/sim_config.step_size));
%     udelayTraj = zeros(2,ceil(duration/sim_config.step_size));
end
delaysteps = sim_config.delaysteps;
n = plant.n;
x = x_xhat_u_d(1:n);
xhat = x_xhat_u_d(n+1:2*n);
u_e_dist_estDist_filter = x_xhat_u_d(2*n+1:2*n+7);
distEst_filtered = x_xhat_u_d(2*n+6:2*n+7);

% ----------------- update the estimation of uncertainty -----------------
xtilde = xhat-x;
if mod(t,distEst_config.Ts) == 0
    Bsigmahat = distEst_config.adapt_gain*xtilde; 
    distEst = plant.Bpinv_fcn(x)*Bsigmahat;
    if controller.distEstScheme == 3
        distEst = distEst - distLearned(x);  % estimating the reminder uncertainty
    end
%     uehold = ue; 
end
% ------------------------------------------------------------------------

% ----------------- use variable step -----------------
% xtilde = xhat-x;
% distEst = 0.92*dist_config.dist_fcn(t,x);
% Bsigmahat = plant.B_fcn(x)*distEst;
% ------------------------------------------------------------------------

% tic;
if controller.filter_distEst == 1
    distEst_ccm_control = distEst_filtered;
else
    distEst_ccm_control = distEst;
end
if controller.distEstScheme == 3
    distEst_ccm_control = distEst_ccm_control +  distLearned(x);
end

ue = ccm_law(t,x,plant,controller,distLearned,distEst_ccm_control,distEst_config.est_errBnd);
uTrajin(:,round(t/sim_config.step_size+1)) = ue(1:2);

% toc;
% u = ue(1:end-1); 
wt = dist_config.dist_fcn(t,x);

% ------------------- update state predictor -----------------------------
xdot = plant.f_fcn(x)+plant.B_fcn(x)*ue(1:end-1); % nominal dynamics;
if controller.use_distModel_in_planning_control == 1 && controller.distEstScheme == 1
        xdot = xdot  + plant.B_fcn(x)*distLearned(x);
end
xhatdot = xdot + Bsigmahat - distEst_config.a_pred*xtilde;
% ------------------------------------------------------------------------
if round(t/sim_config.step_size) < delaysteps
    udelay = [0;0];
else 
    udelay = uTrajin(:,1+max(0,round(t/sim_config.step_size-delaysteps))); % nominal dynamics;
end
udelayTraj(:,round(t/sim_config.step_size+1)) = udelay;
xdot = plant.f_fcn(x)+plant.B_fcn(x)*udelay; %

% if controller.use_distModel_in_planning_control == 1 && controller.distEstScheme == 1
%         xdot = xdot  + plant.B_fcn(x)*distLearned(x);
% end
% xhatdot = xdot + Bsigmahat - distEst_config.a_pred*xtilde;

% update the states of actual system, state predictor, ...
% if round(t/sim_config.step_size) < delaysteps
%     dxdt = [xdot; xhatdot; -Klp.*u_e_dist_estDist_filter]+[zeros(n,1); zeros(n,1); Klp.*[[0;0];ue(end);wt;distEst]];
% else 
%     dxdt = [xdot; xhatdot; -Klp.*u_e_dist_estDist_filter]+[zeros(n,1); zeros(n,1); Klp.*[uTrajin(:,1+max(0,round(t/sim_config.step_size-delaysteps)));ue(end);wt;distEst]];
% end

dxdt = [xdot; xhatdot; -Klp.*u_e_dist_estDist_filter]+[zeros(n,1); zeros(n,1); Klp.*[ue;wt;distEst]];

if sim_config.include_dist == 1
   dxdt(1:n,:) = dxdt(1:n,:) + plant.B_fcn(x)*wt;
end
if t >= duration-sim_config.step_size
     assignin('base','uTrajin',uTrajin);
%      assignin('base','udelayTraj',udelayTraj);
end

end

function [intensity,distance_to_center] = dist_distribution(X,Z,center,radius)
distance_to_center = sqrt((X-center(1)).^2 + (Z-center(2)).^2);

% --------------- using a cosine function ------------------
% intensity = zeros(size(distance_to_center));
% intensity(distance_to_center>radius) = 0;
% intensity(distance_to_center<=radius-1) = 1;
% tmp = distance_to_center>radius-1 & distance_to_center<=radius;
% intensity(tmp) = (cos(pi*(radius-distance_to_center(tmp)-1))+1)/2;

% -------------- using a ReLU function ---------------------
% intensity = max(radius- distance_to_center,0)./radius;

% ---------------using a linear function -------------------
% intensity = distance_to_center./radius;

% --------------- using an inverse function ----------------
intensity = 1./(distance_to_center.^2+1);

% --------------- using an inverse function 2 ----------------
% intensity = 1./(sqrt(distance_to_center.^2)+1);
end

% ---------------- True disturbance --------------------
function dist_force = actual_dist_fcn(t,x,center,radius)
% compute the disturbance force given x
% x is a n by m matrix, where each column represents a state vector value 
max_damping = 0.5;

[dist_intensity,~] = dist_distribution(x(1,:),x(2,:),center,radius);
% dist_intensity = 0.3;
dist_force_max = (x(4,:)^2+x(5,:)^2)*max_damping;
dist_force = [-1; -1]*(dist_intensity.*dist_force_max); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% changed %%%%%%%%%%%%%%
% dist_force = -1./(1+exp(-5*phi))*0.1*(x(4)^2+x(5)^2);
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
% 
% x_pt1 = [[e; -e]+x(:,1), [1;1]*x(:,2:4)];
% x_pt2 = [[1;1]*x(:,1), [e; -e]+x(:,2), [1;1]*x(:,3:4)];
% x_pt3 = [[1;1]*x(:,1:2), [e; -e]+x(:,3), [1;1]*x(:,4)];
% x_pt4 = [[1;1]*x(:,1:3), [e; -e]+x(:,4)];
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

function [dist_force, d_force_dx]= perfect_learner(x,center,radius)
% compute the disturbance force given x
% x is a n by m matrix, where each column represents a state vector value 
max_damping = 0.5;
[dist_intensity,distance_to_center] = dist_distribution(x(1,:),x(2,:),center,radius);
dist_force_max = (x(4,:).^2+x(5,:).^2)*max_damping;
dist_force = [-0.8;-1]*(dist_intensity.*dist_force_max); 

if nargout>1
    [n,N]  = size(x);
    d_force_dx = zeros(2,n,N);    
    for i = 1:N        
        % ------- using an inverse function: inversely proportional to squared distance ------------
        d_force_dx(:,1:2,i) = [-1;-1]*(-2/(distance_to_center(i)^2+1)^2*dist_force_max(i)*(x(1:2,i)-center)');
        d_force_dx(:,4:5,i) = [-1;-1]*(dist_intensity(i)*2*max_damping*x(4:5,i)');
        
        % ---------------- using an inverse function 2: inversely proportional to distance-----------
%         d_force_dx(:,1:2,i) = [-1;-1]*(-1/(distance_to_center(i)+1)^2/distance_to_center(i)*dist_force_max(i)*(x(1:2,i)-center)');
%         d_force_dx(:,4:5,i) = [-1;-1]*(dist_intensity(i)*2*max_damping*x(4:5,i)');
    end
end
end



function [dist_force, d_force_dx]= zero_dist(x)
[n,N]  = size(x);
dist_force = zeros(2,N); 
if nargout>1
    d_force_dx = zeros(2,n,N);    
end
end