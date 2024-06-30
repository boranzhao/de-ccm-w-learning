
function ue= robust_ccm_law(t,x,plant,controller,distLearned,distEst,distEst_errBnd)
persistent t_pre beq_pre copt_pre Erem_pre

if isempty(t_pre) || (t == 0 && t_pre ~=0)
    t_pre = -3;
    Erem_pre = Inf;
end
geodesic = controller.geodesic; 
x_nom = controller.x_nom_fcn(t);
u_nom = controller.u_nom_fcn(t);
if isempty(geodesic) 
    % when the CCM metric is constant and the geodesic is a straight line 
%     gamma ;
    gamma = [x_nom x];
    gamma_s = x-x_nom;
    Erem = gamma_s'*controller.M*gamma_s; 
    gamma_s1_Mx = gamma_s'*controller.M;
    gamma_s0_Mxnom = gamma_s1_Mx;
else 
    n = plant.n; N = geodesic.N; D = geodesic.D; 
    if isempty(beq_pre)
        beq_pre = zeros(2*plant.n,1);
        copt_pre = zeros(plant.n*(geodesic.D+1),1);
    end
    %------------ for testing -----------------
    % Erem = 0;
    % ue = [u;Erem];
    % return;
    %------------------------------------------

    beq = [x_nom;x];    
    % get the initial value of c corresponding to a straight line
    c0 = zeros(n*(D+1),1);
    %     for i =1:n    
    %         c0((i-1)*(D+1)+1,1) = xStar(i);
    %         c0((i-1)*(D+1)+2,1) = x(i)- xStar(i);    
    %     end
    % vectorized format to improve computational efficiency
    i =1:n;    
    c0((i-1)*(D+1)+1,1) = x_nom;
    c0((i-1)*(D+1)+2,1) = x - x_nom; 

    % tic;
    if norm(beq-beq_pre)<1e-8 && ~isinf(Erem_pre)
        copt = copt_pre;
        Erem = Erem_pre;
    else
        % ----------------- use OPTI -----------------------------------------
        % Opt = opti('fun',geodesic.costf,'grad',geodesic.grad,'eq',geodesic.Aeq,beq,'ndec',geodesic.ndec,'x0',c0,'options',geodesic.opts_opti);
        % [copt,Erem,exitflag,info] = solve(Opt,c0);
        %     nlprob = convIpopt(Opt.prob,geodesic.opts_opti);    
        %     nlprob = convMatlab(Opt.prob,geodesic.opts_opti); 
        % --------------------------------------------------------------------

        % --------------- ipopt ----------------------------------------------
        % geodesic.nlprob.options.rl = beq;
        % geodesic.nlprob.options.ru = beq;
        % geodesic.nlprob.x0 = c0;
        % [copt,Erem,exitflag,info] = opti_ipopt(geodesic.nlprob,c0);
        % --------------------------------------------------------------

        % ---------------- matlab -----------------------
        geodesic.nlprob.beq = beq;
        geodesic.nlprob.x0 = c0;
%         tic;
        [copt,Erem,exitflag,info] = fmincon(geodesic.nlprob);
%         toc;
        if exitflag<0
            disp('geodesic optimization problem failed!');
        end
        % ------------------------------------------------
        beq_pre = beq;
        copt_pre = copt;
        Erem_pre = Erem;
    end
    % toc;
    % ----------------- compute the control law -----------------------
    %     tic;
    %     gamma = zeros(n,N+1);
    %     gamma_s = zeros(n,N+1);  
    %     for i = 1:n   
    %        gamma(i,:) = copt((i-1)*(D+1)+1:i*(D+1),:)'*T;       % gamma(i) is 1*(N+1); the ith elment of gamma on all the (N+1) nodes
    %        gamma_s(i,:) = copt((i-1)*(D+1)+1:i*(D+1),:)'*T_dot;
    %     end  
    %     toc;
    % vectorized format (more computationally efficient)
    copt = transpose(reshape(copt,D+1,n)); % the ith row corresponds to the ith element
    gamma = copt*geodesic.T;
    gamma_s = copt*geodesic.Tdot;
% ----------------------------------------------------------------

% % -------- verify whether the curve found is really a geodesic ----------
% % according to equation (11) in Leung  & Manchester
% error = 0;
% for k=1:N+1
%     error = error + (gamma_s(:,k)'*(controller.W_fcn(gamma(:,k))\gamma_s(:,k))-Erem)^2*geodesic.w_cheby(k);
% end
% error = sqrt(error)/Erem;
% if error>=1e-5
% %     disp('The curve optimized is probably not a geodesic!');
%     fprintf(1,'t= %.2e, Error = %.3e, the curve optimized is probably not a geodesic!\n',t,error);
%     if error> 1e-2
%         pause;
%     end
% end
% % -----------------------------------------------------------------------
gamma_s1_Mx = gamma_s(:,end)'/controller.W_fcn(x);
gamma_s0_Mxnom = gamma_s(:,1)'/controller.W_fcn(x_nom);
end
% tic;
plant_fx = plant.f_fcn(x);
plant_fx_nom =  plant.f_fcn(x_nom);

if controller.ccm_law_form == CtrlDesignOpts.ccm_integration && controller.ccm_mat_ineq_form == ...
    CtrlDesignOpts.ccm_mat_ineq_use_rho
    warning('Currently does not support the integration-form control law! Min-norm control law is selected.');
end

if controller.use_distModel_in_planning_control == 0
    % without learned dynamics
    phi0 = gamma_s1_Mx*(plant_fx + plant.B*u_nom) - ...
    gamma_s0_Mxnom*(plant.f_fcn(x_nom) + plant.B*u_nom) + ...
    controller.lambda*Erem;
%     phi0 = gamma_s1_Mx*(plant_fx) - ...
%     gamma_s0_Mxnom*(plant.f_fcn(x_nom) + plant.B*u_nom) + ...
%     controller.lambda*Erem;
else
    % with learned dynamics
%     phi0 = gamma_s1_Mx*(plant_fx + plant.B*(u_nom+distLearned(x))) - ...
%     gamma_s0_Mxnom*(plant.f_fcn(x_nom) + plant.B*(u_nom+distLearned(x))) + ...
%     controller.lambda*Erem;  % original;
   
    % ------------ updated ---------------------
    if controller.distEstScheme == 0 ||  controller.distEstScheme == 1
        utmp = u_nom+distLearned(x);
    elseif controller.distEstScheme == 2 
        utmp = u_nom;
    end
    phi0 = gamma_s1_Mx*(plant_fx + plant.B*utmp) - ...
    gamma_s0_Mxnom*(plant.f_fcn(x_nom) + plant.B*(u_nom+distLearned(x_nom))) + ...
    controller.lambda*Erem; 
%     phi0 = gamma_s1_Mx*(plant_fx) - ...
%     gamma_s0_Mxnom*(plant.f_fcn(x_nom) + plant.B*(u_nom+distLearned(x_nom))) + ...
%     controller.lambda*Erem; 
  % --------------------------------------------------
end
if controller.distEstScheme ~= 0
    if controller.use_distEst_errBnd == 1
        phi0 = phi0 + norm(gamma_s1_Mx*plant.B)*distEst_errBnd; 
%         phi0 = phi0 + norm(gamma_s1_Mx)*distEst_errBnd; 
    end
end
if phi0 <=0
    u = u_nom;
else
    phi1 = gamma_s1_Mx*plant.B;
    u = u_nom - phi0*phi1'/(phi1*phi1'+1e-8);
end

% if phi0 <=0
%     u = [0;0];
% else
%     phi1 = gamma_s1_Mx*plant.B;
%     u = [0;0] - phi0*phi1'/(phi1*phi1'+1e-12);
% end

ue = [u;Erem];
if (t-t_pre>= 0.4) && mod(t,1)< 0.1
    fprintf('t = %.1f s\n',t);
    t_pre = t;
end
% toc;
end
