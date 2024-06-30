% MAIN  --  Planar rotor  --  Minimal-Force trajectory
%
% Fin the minimal torque-squared trajectory to move the quad-rotor from an
% arbitrary state to the origin.

function soln = trajOpt_pvtol(plant,trajGen_config,initGuess,stateCst)

x0 = trajGen_config.x0;
xF = trajGen_config.xF;
duration =  trajGen_config.duration;
u_bnd  = trajGen_config.u_bnd;
x_bnd = trajGen_config.x_bnd; 
include_dist_model = trajGen_config.include_dist_model;
dist_model = trajGen_config.dist_model;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u) dyn_fcn(t,x,u,plant.f_fcn,plant.B,include_dist_model,dist_model); % add some disturbances later
% problem.func.dynamics = @(t,x,u) pvtolDynamics(t,x,u,plant); % add some disturbances later
problem.func.pathObj = @(t,x,u) pathObjective(u);  %Force-squared cost function
problem.func.bndObj = @(t0,x0,tF,xF) bndObjective(t0,x0,tF,xF); % Use cost function to define the terminal constraint
if nargin >= 4
    problem.func.pathCst = @(t,x,u) stateCst(x);
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration-5;
problem.bounds.finalTime.upp = duration+5;

problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
% problem.bounds.finalState.low = -inf*ones(6,1);
% problem.bounds.finalState.upp = inf*ones(6,1);
problem.bounds.finalState.low = xF;
problem.bounds.finalState.upp = xF;

problem.bounds.state.low = x_bnd(:,1);
problem.bounds.state.upp = x_bnd(:,2);

problem.bounds.control.low = u_bnd(:,1);
problem.bounds.control.upp = u_bnd(:,2);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory: a straightline          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.guess.time = initGuess.time;
problem.guess.state = initGuess.state;
problem.guess.control = initGuess.control;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.options.nlpOpt = optimset(...
    'Display','iter','DerivativeCheck','off',...
    'MaxFunEvals',1e6,'MaxIter',1000,'TolFun',1e-4,'TolCon',1e-5,...
    'GradConstr','on','GradObj','on');
% optimoptions('fmincon',...
%     'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,...
%     'Display','iter','ConstraintTolerance',1e-6,'OptimalityTolerance',1e-4,...
%     'MaxFunctionEvaluations',1e6);


% problem.options.method = 'trapezoid'; 
% problem.options.trapezoid.nGrid = 80;

problem.options.method = 'hermiteSimpson';  
problem.options.hermiteSimpson.nSegment = 60;


% problem.options.method = 'chebyshev';  
% problem.options.verbose = 3; % How much to print out?
% problem.options.chebyshev.nColPts = 10;  %method-specific options

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln = optimTraj(problem);
% save('TrajOpt','soln','x0','xF','duration')

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Unpack the simulation
dt = 0.001;
ts = soln.grid.time(1):dt:soln.grid.time(end); 
states = soln.interp.state(ts);
inputs = soln.interp.control(ts);
% %-----------------------------------------------------------------------
return;

%% Plots:
px = states(1,:);
pz = states(2,:);
q  = states(3,:);
vx = states(4,:);
vz = states(5,:);
dq = states(6,:);

u1 = inputs(1,:);
u2 = inputs(2,:);
% %--------- apply the open-loop control law to simulate the system---------
% N = length(t);
% xt = x0;
% % [ts,states] = ode23s(@(t,x)  plant.f_fcn(x) + plant.B*soln.interp.control(t),[0  duration],x0); 
% states = states';
% px = states(1,:);
% pz = states(2,:);
% q = states(3,:);
% vx = states(4,:);
% vz = states(5,:);
% dq = states(6,:); 


close all;
figure(1); clf;

subplot(2,2,1); hold on;
plot(ts,px);
xlabel('t')
ylabel('p_x (m)')
title('Minimum force-squared trajectory')

subplot(2,2,2); hold on;
plot(ts,pz);
xlabel('t')
ylabel('p_z (m)')

subplot(2,2,3); hold on;
plot(ts,q*180/pi);
xlabel('t')
ylabel('\phi (deg)')

subplot(2,2,4); hold on;
plot(ts,u1);  plot(ts,u2);
xlabel('t')
ylabel('u (N)')
legend('u_1','u_2');


figure;
subplot(2,2,1);
plot(ts,vx);
xlabel('t')
ylabel('v_x (m/s)');

subplot(2,2,2);
plot(ts,vz);
xlabel('t')
ylabel('v_z (m/s)');

subplot(2,2,3);
plot(ts,dq*180/pi);
xlabel('t')
ylabel('$\dot{\phi}$ (degree/s)','interpreter','latex');
% 
% function dx_dt = pvtolDynamics(t,x,u,plant)
% N = size(x,2);
% dx_dt = zeros(6,N);
% dx_dt(1:3,:) = x(4:6,:);
% dx_dt(4,:) = -1/plant.m*(u(1,:)+u(2,:)).*sin(x(3,:));
% dx_dt(5,:) = 1/plant.m*(u(1,:)+u(2,:)).*cos(x(3,:))-plant.g;
% dx_dt(6,:) = (u(1,:)-u(2,:))*plant.l/plant.J;
% end

pause;
return;
end
function [obj, objGrad] = pathObjective(u)
% [obj, objGrad] = pathObjective(u)
%
% Computes the objective function (and gradients)
obj = sum(u.^2*1);
if nargout == 2  % Analytic gradients
    nTime = length(u);    
    objGrad = zeros(9,nTime); %4 = [time + 6 states + 2 inputs];    
    objGrad(8:9,:) = 2*u;  %gradient obj wrt u
end
end

function [dx, dxGrad] = dyn_fcn(t,x,u,f_fcn,B,include_dist_model,dist_model)
if nargin<6
    include_dist_model = 0;
    dist_model = @(x) [0 0]';
end
if include_dist_model == 0
    dx = f_fcn(x)+B*u;
else
    [dist_force,dforce_dx] = dist_model(x);
    dx = f_fcn(x)+B*(u+dist_force); 
end
if nargout == 2   % Analytic gradients
    nTime = length(u);
    dxGrad = zeros(6,9,nTime); %4 = [time + 6 states + 2 inputs];
    for i = 1:nTime
        if include_dist_model == 0
            dxGrad(1:6,2:7,i) = df_dx(x(:,i));
        else
            dxGrad(1:6,2:7,i) = df_dx(x(:,i)) + B*dforce_dx(:,:,i);
        end
        dxGrad(1:6,8:9,i) = B;
    end
end
end

function [J, JGrad] = bndObjective(t0,x0,tF,xF)
    J = tF*5;
    if nargout >=2        
        JGrad = zeros(1,14);  % 2 time instants + 12 states
        JGrad(8) = 5;        
    end
end