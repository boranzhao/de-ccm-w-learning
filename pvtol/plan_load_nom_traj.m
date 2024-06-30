gray_color = [1 1 1]*80/255;
file_traj = ['nomTraj_x0xF_config_' num2str(x0xF_config)];
if use_distModel_in_planning_control == 1
    file_traj = [file_traj '_w_learning.mat'];  
else
    file_traj = [file_traj '_no_learning.mat'];  
end 
if sim_config.replan_nom_traj == 1
    trajGen_config.x0 = x0;
    trajGen_config.xF = xF;
    trajGen_config.x_bnd = x_bnd;
    trajGen_config.u_bnd = u_bnd;
    trajGen_config.include_obs = sim_config.include_obs;
    trajGen_config.include_tube = sim_config.include_tube;
    trajGen_config.tube_xz = tube_xz;
    trajGen_config.duration = duration;
    trajGen_config.include_dist_model = use_distModel_in_planning_control;
    trajGen_config.dist_model = distLearned;

    % ------------------------ Specify the obstacles-----------------------
%     obs = [3.8 5 0.8;           
%             6.2 5 0.8];        
    obs = [3.1 6 0.6;           
    6.9 6 0.6;
    5 3 0.5]; %     5 3 0.5
    trajGen_config.obs = obs;
           
    figure(1);clf;hold on;    
    % visualize the area with disturbances
    xx = 0:0.05:11;
    zz = 0:0.05:11;
    [X,Z] = meshgrid(xx,zz);
    Dist_intensity= dist_distribution(X,Z,dist_config.center,dist_config.radius);
    Dist_distribution.X = X;
    Dist_distribution.Z = Z;
    Dist_distribution.intensity = Dist_intensity;
    visualize_dist_area(Dist_distribution);
    if sim_config.include_obs == 1
        visualize_obs(obs,gray_color);
    end
    xlim([0 11]);
    ylim([0 11]);
    trajGen_config.obs = obs;
    soln = plan_traj_pvtol(plant,trajGen_config);

    tF = soln.grid.time(end); trajGen_config.tF = tF;
    save(file_traj,'trajGen_config','soln','Dist_distribution');
else
    load(file_traj);
%     load('simulation_results_CDC/safe_exploration/CDC_RD_CCM_T_0.0001_lam_0.8_w_dist_1_with_XX_Adam_bound0.1_00_810_w_obs.mat');
end
duration = trajGen_config.tF;   % modify the duration according to the computed trajectory
% duration = 3;

%% show the planned traj 
x_nom_fcn = soln.interp.state;
u_nom_fcn = soln.interp.control;
times = 0:0.05:duration;
simuLen = length(times);
xnomTraj = zeros(n,simuLen);
unomTraj = zeros(nu,simuLen);
for t =1:simuLen
    xnomTraj(:,t) = x_nom_fcn(times(t));
    unomTraj(:,t) = u_nom_fcn(times(t));
end
figure(1);clf
hold on;

% visualize the area with disturbances
visualize_dist_area(Dist_distribution);

plot(xnomTraj(1,:),xnomTraj(2,:),'linewidth',1);
if trajGen_config.include_obs == 1
    visualize_obs(trajGen_config.obs,gray_color);
end    
sim_config.trajGen_config = trajGen_config;
figure(2);
subplot(2,1,1)
plot(times, xnomTraj(4,:),times, xnomTraj(5,:));
legend('v_x','v_z');
subplot(2,1,2)
plot(times, unomTraj(1,:),times, unomTraj(2,:));
legend('u_1','u_2');
