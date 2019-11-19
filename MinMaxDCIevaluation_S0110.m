function [tp_S010] = MinMaxDCIevaluation_S0110(x,s1,s2)
%% Input: 1. x gene from ga
%% Output:1. best anatomy in structure generated
%%        2. plots of random structure vs best DCI structure

%% DCI evaluation is conducted using ga in. The resulted anatomies are 
%% evaluated using plots of the Mb(t2)|tp1,tp2
%% xacro file for robot build is: 
%  /home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/kinematic_verification_011.xacro
%% parent matlab file is:
%  /modular_dynamixel/raad2020/InvestigateAsadaTheorem1.m
% form this file GlobalMinMaxDCI_S0110 is called for user-specified weights

%% Kinematic based on random values
R010_rand = [0.5 0.5 -0.5]';
P010_rand = [0.1 -0.1 0.1]';
tp_rand = [1.2479; -0.8741];

n = 2; % Structural Block number
[S2] = Build_SB110_forGMinMaxDCI(s1,s2,R010_rand,P010_rand,n);
xi_ai_rand(:,1) = s1.xi;
xi_ai_rand(:,2) = S2.xi(:,3);
xi_pi_rand(:,1) = S2.xi(:,1);
xi_pi_rand(:,2) = S2.xi(:,2);

gst0_rand = S2.Cg;
gsCoMi0_rand = S2.g_s_CoM0;

%% Kinematic based on values extracted by ga call in InvestigateAsadaTheroem1.m
% Kinematics are based to file modular_dynamixel/GlobalMinMaxDCI_S010.m
R0110 = x(2:4);
P0110 = x(5:7);
ci = x(8:9);
step_angle = deg2rad(5);
tpi0 = -1.5708;
for k=1:2
    tp_S010(k) = step_angle * (ci(k)-1) + tpi0;
end
n = 2; % Structural Block number
[S2] = Build_SB110_forGMinMaxDCI(s1,s2,R0110,P0110,n);
xi_ai(:,1) = s1.xi;
xi_ai(:,2) = S2.xi(:,3);
xi_pi(:,1) = S2.xi(:,1);
xi_pi(:,2) = S2.xi(:,2);
gst0 = S2.Cg;
gsCoMi0 = S2.g_s_CoM0;

%% Plots for Random vs best S010
[M_b_11,M_b_12,M_b_22]    = BestMinMaxDCIanatomy_S0110_MmatrixElements(xi_ai,xi_pi,gsCoMi0,s2.Mbi);
[M_br_11,M_br_12,M_br_22] = BestMinMaxDCIanatomy_S0110_MmatrixElements(xi_ai_rand,xi_pi_rand,gsCoMi0_rand,s2.Mbi);

[tp1f,tp2f,t2f] = meshgrid(-1.5708:deg2rad(15):1.5708,-1.5708:deg2rad(15):1.5708,-2:0.08:2);

figure; % M11
cmin = 1.0e-02;
cmax = 3.0e-02;
scatter3(tp1f(:),tp2f(:),t2f(:),50,M_b_11(:),'filled','MarkerFaceAlpha',0.1); hold on; 
title('M_{11}'); xlabel('θ_{p1}'); ylabel('θ_{p2}'); zlabel('θ_{2}'); colorbar; set(gca, 'CLim', [cmin cmax]);
M11cut = M_b_11;
ind = find(abs(M11cut)>2.5e-02);
f1 = M11cut;
f1(ind)=NaN;
scatter3(reshape(tp1f,[1 prod(size(tp1f))]),reshape(tp2f,[1 prod(size(tp1f))]),reshape(t2f,[1 prod(size(tp1f))]),50,reshape(f1,[1 prod(size(tp1f))]),'filled'); colorbar; set(gca, 'CLim', [cmin cmax]);
hold on;

figure; % M12
cmin = 1.0e-04;
cmax = 3.0e-03;
scatter3(tp1f(:),tp2f(:),t2f(:),50,M_b_12(:),'filled','MarkerFaceAlpha',0.1); hold on; 
title('M_{12}'); xlabel('θ_{p1}'); ylabel('θ_{p2}'); zlabel('θ_{2}'); colorbar; set(gca, 'CLim', [cmin cmax]);
M12cut = M_b_12;
ind = find(abs(M12cut)>7.0e-04);
f1 = M12cut;
f1(ind)=NaN;
scatter3(reshape(tp1f,[1 prod(size(tp1f))]),reshape(tp2f,[1 prod(size(tp1f))]),reshape(t2f,[1 prod(size(tp1f))]),50,reshape(f1,[1 prod(size(tp1f))]),'filled'); colorbar; set(gca, 'CLim', [cmin cmax]);
hold on;

% figure; % M22
% scatter3(tp1f(:),tp2f(:),t2f(:),50,M_b_22(:),'filled','MarkerFaceAlpha',0.25); hold on; 
% title('M_{22}'); xlabel('θ_{p1}'); ylabel('θ_{p2}'); zlabel('θ_{2}'); colorbar;
% M22cut = M_b_22;
% ind = find(abs(M22cut)>0.005);
% f1 = M22cut;
% f1(ind)=NaN;
% scatter3(reshape(tp1f,[1 prod(size(tp1f))]),reshape(tp2f,[1 prod(size(tp1f))]),reshape(t2f,[1 prod(size(tp1f))]),25,reshape(f1,[1 prod(size(tp1f))]),'filled');
% hold on;
end