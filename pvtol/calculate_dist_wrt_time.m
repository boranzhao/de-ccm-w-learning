% calculate disturbance wrt time
clear;
close all;

% Task1: 00_810
dt = 0.0002;
load('simulation_results/sim_de_ccm_3__T_0.0002_lam_0.8_w_dist_1_with_poor_Adam_bound0.1_00_810_w_obs.mat')
params = load('params_poor.mat').params;
prd_dist = @(x) -uncer_func_poor(x,params);

dist_learned = learned_dist_fcn(xTraj, prd_dist);
dist_true = actual_dist_fcn(xTraj, [5,5], 4);

hold on
h2 = plot(times,dist_learned(1,:),'b-.','Linewidth',1);
h3 = plot(times,estDistTraj(1,:),'r--','Linewidth',1);
h1 = plot(times,dist_true(1,:),'k--','Linewidth',1.5);

% load('simulation_results/sim_ccm_lam_0.8_w_dist_1_00_810_w_obs.mat')
% 
% dist_learned = learned_dist_fcn(xTraj, prd_dist);
% dist_true = actual_dist_fcn(xTraj, [5,5], 4);
% h4 = plot(times,dist_true(1,:),'k-','Linewidth',1);
% h5 = plot(times,0*times,'b-','Linewidth',1);
% h6 = plot(times,estDistTraj(1,:),'r-.','Linewidth',1.5);

axis square
xlabel('Time (s)','interpreter','latex')
ylabel('Disturbance','interpreter','latex')
legend([h1,h2,h3,h4,h5,h6],{'Poor L: $d_1$', 'Poor L: $\hat d_1$', 'Poor L: $\check d_1$', 'No L: $d_1$', 'No L: $\hat d_1$', 'No L: $\check d_1$'},'NumColumns',2,'Location','southeast','Orientation','vertical','interpreter','latex');

xlim([0 10.5]);
% ylim([-0.7 0.05]);
goodplot([6 5]);

% print('With partial learning_ccm fails.pdf', '-painters', '-dpdf', '-r150');
print('Plot disturbance_test.pdf', '-painters', '-dpdf', '-r150');
%% some functions
function [dist_force, d_force_dx]= learned_dist_fcn(x,prd_dist)
[n,N]  = size(x);

d_force_dx = zeros(2,n,N);
% dist_force = zeros(2,N);

x_reduced = x([1 2 4 5],:)';
dist_force = prd_dist(x_reduced);
if nargout>1
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
function dist_force = actual_dist_fcn(x,center,radius)
% compute the disturbance force given x
% x is a n by m matrix, where each column represents a state vector value 
max_damping = 0.5;

[dist_intensity,~] = dist_distribution(x(1,:),x(2,:),center,radius);
dist_force_max = (x(4,:).^2+x(5,:).^2)*max_damping;
dist_force = [-1; -1]*(dist_intensity.*dist_force_max); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% changed %%%%%%%%%%%%%%
% dist_force = -1./(1+exp(-5*phi))*0.1*(x(4)^2+x(5)^2);
end