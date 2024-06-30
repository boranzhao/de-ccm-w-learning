function [soln] =plan_traj_pvtol(plant,trajGen_config)
x0 = trajGen_config.x0;
xF = trajGen_config.xF;
duration = trajGen_config.duration;
% u_bnd = trajGen_config.u_bnd;
% x_bnd = trajGen_config.x_bnd;
% dist_model = trajGen_config.dist_model;
% include_dist_model = trajGen_config.include_dist_model;

% initial guess of the solution: important to find a feasible solution    
initGuess.time = [0,duration/2 duration];
initGuess.state = [x0, (x0+xF)/2 xF];
initGuess.control = plant.g*plant.m*ones(2,3)/2;
if trajGen_config.include_obs == 1
    % ----- formulate the state constraints for collision avoidance 
    if trajGen_config.include_tube == 1
        tube_xz = trajGen_config.tube_xz;
    else
        tube_xz = 0;
    end
    stateCst = @(x)  cst_avoid_obstacle(x,trajGen_config.obs,tube_xz,plant.nu);

    initGuess.time = [0, duration/2 duration];
    initGuess.state = [x0, [5 5 zeros(1,4)]'  xF]; %8 5 zeros(1,4)]'
    initGuess.control = plant.g*plant.m*ones(2,length(initGuess.time))/2;        
    
    soln = trajOpt_pvtol(plant,trajGen_config,initGuess,stateCst);
else
    soln = trajOpt_pvtol(plant,trajGen_config,initGuess);
end    
fprintf(1,'cost = %.2f\n',soln.info.objVal);
