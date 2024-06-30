syms x z dx dz
c1 = 5;
c2 = 5;
r = 4;

distance_to_center = sqrt((x-c1)^2 + (z-c2)^2);

intensity = 1/(distance_to_center^2+1);

max_damping = 0.5;

dist_force_max = (dx^2+dz^2)*max_damping;
dist_force = [-0.8; -1]*(intensity*dist_force_max); 

jacb = jacobian(dist_force, [x;z;dx;dz]);
jacb

% test some points
xx = [4 4 1 1];
x = xx(1);
z = xx(2);
dx = xx(3);
dz = xx(4);
subs(jacb')

%% test jacobian

% Ziyao
params = load('params.mat').params;
prd_dist = @(x) -uncer_func(x,params);

prd_dist(xx);
jacobian(prd_dist,xx)



function jac = jacobian(prd_dist,x)
% only works for 2x4 jacobian
% x is 1x4
jac = zeros(2,4);
e = 1e-6;
x_pt1 = [[e; -e]+x(:,1), [1;1]*x(:,2:4)];
x_pt2 = [[1;1]*x(:,1), [e; -e]+x(:,2), [1;1]*x(:,3:4)];
x_pt3 = [[1;1]*x(:,1:2), [e; -e]+x(:,3), [1;1]*x(:,4)];
x_pt4 = [[1;1]*x(:,1:3), [e; -e]+x(:,4)];

jac(:,1) = (prd_dist(x_pt1(1,:))-prd_dist(x_pt1(2,:)))/(2*e);    
jac(:,2) = (prd_dist(x_pt2(1,:))-prd_dist(x_pt2(2,:)))/(2*e); 
jac(:,3) = (prd_dist(x_pt3(1,:))-prd_dist(x_pt3(2,:)))/(2*e); 
jac(:,4) = (prd_dist(x_pt4(1,:))-prd_dist(x_pt4(2,:)))/(2*e); 
end