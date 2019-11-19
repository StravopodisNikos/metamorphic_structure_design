function [tp_S010] = MinMaxDCIevaluation_S010(x,s1,s2)
%% Input: 1. x gene from ga
%% Output:1. best anatomy in structure generated
%%        2. plots of random structure vs best DCI structure

%% DCI evaluation is conducted using ga in. The resulted anatomies are 
%% evaluated using plots of the Mb(t2)|tp1
%% xacro file for robot build is: 
%  /home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/kinematic_verification_01.xacro
%% parent matlab file is:
%  /modular_dynamixel/raad2020/InvestigateAsadaTheorem1.m
% form this file GlobalMinMaxDCI_S010 is called for user-specified weights

%% Kinematic based on random values
R010_rand = [0.5 0.5 -0.5]';
P010_rand = [0.1 -0.1 0.1]';
c_rand = 17; % random
step_angle = deg2rad(5);
tpi0 = -1.5708;
tp_rand = step_angle * (c_rand-1) + tpi0;

n = 2; % Structural Block number
[S2] = Build_SB10_forGMinMaxDCI(s1,s2,R010_rand,P010_rand,n);
xi_ai_rand(:,1) = s1.xi;
xi_ai_rand(:,2) = S2.xi(:,2);
xi_pi_rand(:,1) = S2.xi(:,1);
gst0_rand = S2.Cg;
gsCoMi0_rand = S2.g_s_CoM0;

%% Kinematic based on values extracted by ga call in InvestigateAsadaTheroem1.m
% Kinematics are based to file modular_dynamixel/GlobalMinMaxDCI_S010.m
R010 = x(2:4);
P010 = x(5:7);
ci = x(8);
step_angle = deg2rad(5);
tpi0 = -1.5708;
tp_S010 = step_angle * (ci-1) + tpi0;

n = 2; % Structural Block number
[S2] = Build_SB10_forGMinMaxDCI(s1,s2,R010,P010,n);
xi_ai(:,1) = s1.xi;
xi_ai(:,2) = S2.xi(:,2);
xi_pi(:,1) = S2.xi(:,1);
gst0 = S2.Cg;
gsCoMi0 = S2.g_s_CoM0;

%% Plots for Random vs best S010
[M_b_GDCI1_11,M_b_GDCI1_12,M_b_GDCI1_22] = BestMinMaxDCIanatomy_S010_MmatrixElements(xi_ai,xi_pi,gsCoMi0,s2.Mbi);
[M_b_RAND_11,M_b_RAND_12,M_b_RAND_22] = BestMinMaxDCIanatomy_S010_MmatrixElements(xi_ai_rand,xi_pi_rand,gsCoMi0_rand,s2.Mbi);

[t2f,tpf] = meshgrid(-1.5708:0.08:1.5708,-1.5708:deg2rad(5):1.5708);

figure; % M11
cmin = 1.0e-02;
cmax = 3.0e-02;
surf(t2f,tpf,M_b_GDCI1_11,'EdgeColor','flat','FaceColor','interp'); hold on; 
surf(t2f,tpf,M_b_RAND_11,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
title('M_{11}'); xlabel('θ_2'); ylabel('θ_{p1}'); zlabel('M_{11}'); colorbar; set(gca, 'CLim', [cmin cmax]);

figure; % M12
cmin = 1.0e-04;
cmax = 3.0e-03;
surf(t2f,tpf,M_b_GDCI1_12,'EdgeColor','flat','FaceColor','interp'); hold on; 
surf(t2f,tpf,M_b_RAND_12,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
title('M_{12}'); xlabel('θ_2'); ylabel('θ_{p1}'); zlabel('M_{12}'); colorbar; set(gca, 'CLim', [cmin cmax]);

% figure; % M23
% surf(t2f,tpf,M_b_GDCI1_22,'EdgeColor','flat','FaceColor','interp'); hold on; 
% surf(t2f,tpf,M_b_RAND_22,'EdgeColor','interp','FaceColor','texturemap','FaceAlpha',0.5); hold on;
% title('M_{22}'); xlabel('θ_2'); ylabel('θ_{p1}'); zlabel('M_{22}'); colorbar; set(gca, 'CLim', [cmin cmax]);

end