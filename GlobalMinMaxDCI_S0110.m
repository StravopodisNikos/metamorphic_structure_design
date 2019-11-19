function DCI_star = GlobalMinMaxDCI_S0110(x,s1,s2)
% Objective function that minimizes max(DCI) in each generation
% used for modular_dynamixel/raad2020/InvestigateAsadaTheorem1.m
% investigates structures construction for dynamic_synthesis

%% Get chromosome values
step_a2 = x(1); % for 2 DoF
R0110 = x(2:4);
P0110 = x(5:7);
ci = x(8:9); % for S010
wx = x(10:12); % for M12 element default for diagonal elements:10000
%% Determine tpi pseudojoint angles
step_angle = deg2rad(5);
tpi0 = -1.5708;
for k=1:2
    tp(k) = step_angle * (floor(ci(k))-1) + tpi0;
end
%% Searches in the RP Space specified from gene x
% Here the kinematics of the new structure configuration are calculated
% that is: xi_ai',xi_pi',gst0',gsli0'
n = 2; % Structural Block number
[S2] = Build_SB110_forGMinMaxDCI(s1,s2,R0110,P0110,n);
xi_ai(:,1) = s1.xi;
xi_ai(:,2) = S2.xi(:,3);
xi_pi(:,1) = S2.xi(:,1);
xi_pi(:,2) = S2.xi(:,2);
gst0 = S2.Cg;
gsCoMi0 = S2.g_s_CoM0;
%% Sets current anatomy pseudo exponential
P1 = twistexp(xi_pi(:,1),tp(1))*twistexp(xi_pi(:,2),tp(2));
%% for all ti2
p_count = 0;
for ta2=-1.5708:step_a2:1.5708
    q = [0.1 ta2];
    exp_ai(:,:,1) = twistexp(xi_ai(:,1),q(1));
    exp_ai(:,:,2) = twistexp(xi_ai(:,2),q(2));
    p_count = p_count + 1;
    % Compute Link CoM Jacobians
    for i=1:2
        [Jbsli(:,:,i)] = Jbody_CoM_2DoF_forDCI(xi_ai, exp_ai, P1, gsCoMi0, i);
    end
    % Compute mass matrix
    [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,s2.Mbi);
    % DCI
    [DCI(p_count)] = CalculateDynamicConditioningIndex2_2DoF(Mb,wx);  
end

DCI_star = max(DCI); 
end