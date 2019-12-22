function DCI_star = GlobalMinMax_DCI_diag_S0110110(x,s1,s2)
% Objective function that minimizes max(DCI) in each generation
% used for modular_dynamixel/raad2020/Investigate3DoF_MassMatrixRAAD2020.m
% investigates structures construction for dynamic_synthesis of 3 DoF
% This is the first function fos 3DoF structure investigation
% Next is GlobalMinMaxDCI_S010110.m

%% Get chromosome values
step_a23 = x(1); % for 3 DoF
R0110_1 = x(2:4);
P0110_1 = x(5:7);
R0110_2 = x(8:10);
P0110_2 = x(11:13);
ci = x(14:17); 
wx = x(18:20); 
%% Determine tpi pseudojoint angles
step_angle = deg2rad(15);
tpi0 = -1.5708;
for k=1:4
    tp(k) = step_angle * (floor(ci(k))-1) + tpi0;
end
%% Searches in the RP Space specified from gene x
% Here the kinematics of the new structure configuration are calculated
% that is: xi_ai',xi_pi',gst0',gsli0'
n = 2; % Structural Block number
[S2] = Build_SB110_forGMinMaxDCI(s1,s2,R0110_1,P0110_1,n);
xi_ai(:,1) = s1.xi;
xi_ai(:,2) = S2.xi(:,3);
xi_pi(:,1) = S2.xi(:,1);
xi_pi(:,2) = S2.xi(:,2);
%gst0 = S2.Cg;
gsCoM10 = S2.g_s_CoM0;

n = 3; % Structural Block number
[S3] = Build_SB110_forGMinMaxDCI(S2,s2,R0110_2,P0110_2,n);
%xi_ai(:,1) = s1.xi;
xi_ai(:,3) = S3.xi(:,3);
xi_pi(:,3) = S3.xi(:,1);
xi_pi(:,4) = S3.xi(:,2);
gst0 = S3.Cg;
gsCoM20 = S3.g_s_CoM0;

gsCoMi0(:,:,1) = gsCoM10(:,:,1);
gsCoMi0(:,:,2) = gsCoM20(:,:,1);
% gsCoMi0(:,:,3) = gsCoM20(:,:,2);
gsCoMi0(:,:,3) = S3.Cg; % approximation

Mbi(:,:,1) = s2.Mbi(:,:,1);
Mbi(:,:,2) = s2.Mbi(:,:,1);
Mbi(:,:,3) = s2.Mbi(:,:,2);
%% Sets current anatomy pseudo exponential
P1 = twistexp(xi_pi(:,1),tp(1))*twistexp(xi_pi(:,2),tp(2));
P2 = twistexp(xi_pi(:,3),tp(3))*twistexp(xi_pi(:,4),tp(4));
Pi(:,:,1) = P1;
Pi(:,:,2) = P2;
%% for all ti2
p2 = 0;
for ta2=-1.5708:step_a23:1.5708
    p2 = p2+1;
    p3 = 0;
    for ta3=-1.5708:step_a23:1.5708
        p3 = p3+1;
        q = [0.1 ta2 ta3];
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),q(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),q(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),q(3));
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        % DCI
%         wx_offdiag = [0 0 0 wx(1) wx(2) wx(3)];
        wx_diag = [wx(1) wx(2) wx(3) 0 0 0];
        [DCI(p2,p3)] = CalculateDynamicConditioningIndex2_3DoF(Mb,3,wx_diag);
    end
end

DCI_star = max(DCI(:)); 
end