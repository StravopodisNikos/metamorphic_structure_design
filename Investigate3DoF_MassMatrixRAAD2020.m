clc;
clear;
close all;
%% INFO
% Builds 3 DoF Metamorphic structure S0110110

% Investigates Structure synthesis conditions
% 1. calls gamultiobj for DCI_offdiag
% 2.                      && DCI_diag
%%

%% load libraries
% load Murray kinematics
addpath('/home/nikos/matlab_ws/kinematics/robotlinks')
addpath('/home/nikos/matlab_ws/kinematics/screws') 
addpath('/home/nikos/matlab_ws/kinematics/util')
% load Js-Jb calculation function
addpath('/home/nikos/matlab_ws/project_ABBpaper/matlab_files')
% load main folder
addpath('/home/nikos/matlab_ws/modular_dynamixel/')
addpath('/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis')
addpath('/home/nikos/matlab_ws/modular_dynamixel/raad2020')
% load geom3d library
% addpath('/home/nikos/matlab_ws/geom3d')
% addpath('/home/nikos/matlab_ws/geom3d/geom3d')
% addpath('/home/nikos/matlab_ws/geom2d/geom2d')
% addpath('/home/nikos/matlab_ws/geom2d/utils')

%% load zero data for STRUCTURAL BLOCKS
% data are obtained by .m files:
% modular_dynamixel/structural_synthesis/kinematic_verification_C01.m for SB01
% modular_dynamixel/structural_synthesis/kinematic_verification_C011.m for SB011
load('SB10_zero_data_CoM.mat');
pi_10 = pi(:,2:4); % j-i1-k(!)
wi_10 = wi(:,2:3); % j-i1
g0_10 = g0;
gsc_10 = g0(:,:,9); % g_s_lk0
g_lk_TOOL_10 = g0(:,:,10);
xi_10 = xi;
M_10_b = M0_CoM; %6x6x2
load('SB110_zero_data_CoM.mat');
pi_110 = pi(:,2:5); % j1-j2-i1-k(!)
wi_110 = wi(:,2:4); % j1-j2-i1
g0_110 = g0;
gsc_110 = g0(:,:,2); % g_s_lk0
g_lk_TOOL_110 = g0(:,:,14);
xi_110 = xi;
M_110_b = M0_CoM;
%% Initialization of structure geometry
t0 = 0; % first active angle

p1_0 = [0 0 0.0570]';
w1_0 = [0 0 1]';
x1 = createtwist(w1_0,p1_0); %ξa1
% zero tf to the first active frame
g_s_li1_0 = [   1.0000      0         0         0;...
                 0          1.0000    0         0;...
                 0          0         1.0000    0.0570;...
                 0          0         0         1.0000];
% zero tf to the first connection Point of Synthetic Joint
% (zero tf between {s} -> frame
g_s_lk1_0 = [1.0000    0         0          0;...
             0         1.0000    0          0;...
             0         0         1.0000     0.0570;...
             0         0         0          1.0000];
exp1 = twistexp(x1,t0);
f1 = 'pi'; v1 = p1_0;
f2 = 'wi'; v2 = w1_0;
f3 = 'g0'; v3 = g_s_li1_0;
f4 = 'Cg'; v4 = g_s_lk1_0; % only for this, because the first!
f5 = 'expi'; v5 = exp1;
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
f8 = 'gsli1'; v8 = g_s_li1_0;
s1 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8);
% For second string part a check if '010' or '0110' takes place
% f5-f6-f7 are only initialization since 10,110 never start! Values are
% given ONLY inside functions
f1 = 'pi'; v1 = pi_10; 
f2 = 'wi'; v2 = wi_10;
f3 = 'g0'; v3 = g0_10;
f4 = 'Cg'; v4 = g_lk_TOOL_10;
f5 = 'expi'; v5 = exp1; 
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
f8 = 'Mbi'; v8 = M_10_b;
s2 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8);
% For third string part a check if '10' or '110' takes place
% f5-f6-f7 are only initialization since 10,110 never start! Values are
% given ONLY inside functions
f1 = 'pi'; v1 = pi_110; 
f2 = 'wi'; v2 = wi_110;
f3 = 'g0'; v3 = g0_110;
f4 = 'Cg'; v4 = g_lk_TOOL_110;
f5 = 'expi'; v5 = exp1;
f6 = 'xi'; v6 = x1;
f7 = 'Sframe'; v7 = zeros(4);
f8 = 'Mbi'; v8 = M_110_b;
s3 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8);

%% Here call ga's
generations = 500;
population  = 350;
%% 1. Evaluation of structure S0110110
FitnessFunction1 = @(x)[GlobalMinMax_DCI_offdiag_S0110110(x,s1,s3),GlobalMinMax_DCI_diag_S0110110(x,s1,s3)];
nvars1 = 20;
Aeq1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1]; beq1 = 1;
% Aeq1 = []; beq1 = [];
% A1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
A1 = []; b1 = [];
%     step_th23  Rx1      Ry1      Rz1    Px1  Py1  Pz1   Rx2      Ry2      Rz2     Px2  Py2  Pz2   c1  c2  c3  c4  w1  w2  w3
LB1 = [0.05     -1.5708  -1.5708  -1.5708 0    0    0.08  -1.5708  -1.5708  -1.5708 0    0    0.03  1        1        1        1        0.1  0.1  0.1];
UB1 = [0.25     -0.5     -0.5     -0.5    0.15 0.15 0.15  -0.5     -0.5     -0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  13.9999  1    1    1];
LB2 = [0.05     -0.5     -0.5     -0.5    0    0    0.08  -0.5     -0.5     -0.5    0    0    0.03  1        1        1        1        0.1  0.1  0.1];
UB2 = [0.05      0.5      0.5     0.5     0.15 0.15 0.15   0.5      0.5      0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  13.9999  1    1    1];
LB3 = [0.05      0.5      0.5     0.5     0    0    0.08   0.5      0.5     0.5     0    0    0.03  1        1        1        1        0.1  0.1  0.1];
UB3 = [0.25      1.5708   1.5708  1.5708  0.15 0.15 0.15   1.5708   1.5708  1.5708  0.15 0.15 0.15  13.9999  13.9999  13.9999  13.9999  1    1    1];

%% Call gamultiob
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','iter','StallGenLimit',50,'FunctionTolerance',1e-5,'UseParallel', true, 'UseVectorized', false);
% tic;
% [x1,fval1,exitflag1,output1] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],options);
% toc;
% 
% 
% tic;
% [x2,fval2,exitflag2,output2] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB2,UB2,[],options);
% toc;
% 
% tic;
% [x3,fval3,exitflag3,output3] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB3,UB3,[],options);
% toc;

tic;
[x4,fval4,exitflag4,output4] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
toc;
% % save('ResultsInvestigate3DoF_MassMatrixRAAD2020_v2.mat','x4','fval4')
% save('ResultsInvestigate3DoF_MassMatrixRAAD2020_v2.mat','x1','fval1','x2','fval2','x3','fval3','x4','fval4');
eval_x4 = [fval4 x4(:,2:13)]
%% Sort solutions
% range1 = abs(fval1(:,2)-fval1(:,1));
% [best_range1,index1] = sort(range1);
% final_range1 = best_range1(1:floor(size(x1,1))); 
% final_x1 = x1(index1(1:floor(size(x1,1)/2)),:);
% final_sol1 = horzcat(final_range1,final_x1);
% 
% range2 = abs(fval2(:,2)-fval2(:,1));
% [best_range2,index2] = sort(range2);
% final_range2 = best_range2(1:floor(size(x2,1)/2)); 
% final_x2 = x2(index1(1:floor(size(x2,1)/2)),:);
% final_sol2 = horzcat(final_range2,final_x2);
% 
% range3 = abs(fval3(:,2)-fval3(:,1));
% [best_range3,index3] = sort(range3);
% final_range3 = best_range3(1:floor(size(x3,1)/2)); 
% final_x3 = x3(index3(1:floor(size(x3,1)/2)),:);
% final_sol3 = horzcat(final_range3,final_x3); 

% range4 = abs(fval4(:,2)-fval4(:,1));
% [best_range4,index4] = sort(range4);
% final_range4 = best_range4(1:floor(size(x4,1))); 
% final_x4 = x4(index4(1:floor(size(x4,1))),:);
% final_sol4 = horzcat(final_range4,final_x4); 
% % save('ResultsInvestigate3DoF_MassMatrixRAAD2020_SortedSolutions_v2.mat','final_sol4');
% save('ResultsInvestigate3DoF_MassMatrixRAAD2020_SortedSolutions_v2.mat','final_sol1','final_sol2','final_sol3','final_sol4');

%% 2. Evaluation of structure S01010
FitnessFunction2 = @(x)[GlobalMinMax_DCI_offdiag_S01010(x,s1,s2),GlobalMinMax_DCI_diag_S01010(x,s1,s2)];
nvars1 = 18;
Aeq1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1]; beq1 = 1;
% Aeq1 = []; beq1 = [];
% A1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
A1 = []; b1 = [];
%     step_th23  Rx1      Ry1      Rz1    Px1  Py1  Pz1   Rx2      Ry2      Rz2     Px2  Py2  Pz2   c1       c2       w1    w2    w3
LB1 = [0.05     -1.5708  -1.5708  -1.5708 0    0    0.08  -1.5708  -1.5708  -1.5708 0    0    0.03  1        1        0.1   0.1   0.1];
UB1 = [0.25     -0.5     -0.5     -0.5    0.15 0.15 0.15  -0.5     -0.5     -0.5    0.15 0.15 0.15  13.9999  13.9999  1     1     1];
LB2 = [0.05     -0.5     -0.5     -0.5    0    0    0.08  -0.5     -0.5     -0.5    0    0    0.03  1        1        0.1   0.1   0.1];
UB2 = [0.05      0.5      0.5     0.5     0.15 0.15 0.15   0.5      0.5      0.5    0.15 0.15 0.15  13.9999  13.9999  1     1     1];
LB3 = [0.05      0.5      0.5     0.5     0    0    0.08   0.5      0.5     0.5     0    0    0.03  1        1        0.1   0.1   0.1];
UB3 = [0.25      1.5708   1.5708  1.5708  0.15 0.15 0.15   1.5708   1.5708  1.5708  0.15 0.15 0.15  13.9999  13.9999  1     1     1];
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','iter','StallGenLimit',50,'FunctionTolerance',1e-5,'UseParallel', true, 'UseVectorized', false);
tic;
[x5,fval5,exitflag5,output5] = gamultiobj(FitnessFunction2,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
toc;
% range5 = abs(fval5(:,2)-fval5(:,1));
% [best_range5,index5] = sort(range5);
% final_range5 = best_range5(1:floor(size(x5,1))); 
% final_x5 = x5(index5(1:floor(size(x5,1))),:);
% final_sol5 = horzcat(final_range5,final_x5); 
eval_x5 = [fval5 x5(:,2:13)]
%% 3. Evaluation of structure S010110
FitnessFunction3 = @(x)[GlobalMinMax_DCI_offdiag_S010110(x,s1,s2,s3),GlobalMinMax_DCI_diag_S010110(x,s1,s2,s3)];
nvars1 = 19;
Aeq1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1]; beq1 = 1;
% Aeq1 = []; beq1 = [];
% A1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
A1 = []; b1 = [];
%     step_th23  Rx1      Ry1      Rz1    Px1  Py1  Pz1   Rx2      Ry2      Rz2     Px2  Py2  Pz2   c1       c2       c3       w1  w2  w3
LB1 = [0.05     -1.5708  -1.5708  -1.5708 0    0    0.08  -1.5708  -1.5708  -1.5708 0    0    0.03  1        1        1        0.1  0.1  0.1];
UB1 = [0.25     -0.5     -0.5     -0.5    0.15 0.15 0.15  -0.5     -0.5     -0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
LB2 = [0.05     -0.5     -0.5     -0.5    0    0    0.08  -0.5     -0.5     -0.5    0    0    0.03  1        1        1        0.1  0.1  0.1];
UB2 = [0.05      0.5      0.5     0.5     0.15 0.15 0.15   0.5      0.5      0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
LB3 = [0.05      0.5      0.5     0.5     0    0    0.08   0.5      0.5     0.5     0    0    0.03  1        1        1        0.1  0.1  0.1];
UB3 = [0.25      1.5708   1.5708  1.5708  0.15 0.15 0.15   1.5708   1.5708  1.5708  0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','iter','StallGenLimit',50,'FunctionTolerance',1e-5,'UseParallel', true, 'UseVectorized', false);
tic;
[x6,fval6,exitflag6,output6] = gamultiobj(FitnessFunction3,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
toc;
% range6 = abs(fval6(:,2)-fval6(:,1));
% [best_range6,index6] = sort(range6);
% final_range6 = best_range6(1:floor(size(x6,1))); 
% final_x6 = x6(index6(1:floor(size(x6,1))),:);
% final_sol6 = horzcat(final_range6,final_x6);
eval_x6 = [fval6 x6(:,2:13)]
%% 4. Evaluation of structure S011010
FitnessFunction4 = @(x)[GlobalMinMax_DCI_offdiag_S011010(x,s1,s2,s3),GlobalMinMax_DCI_diag_S011010(x,s1,s2,s3)];
nvars1 = 19;
Aeq1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1]; beq1 = 1;
% Aeq1 = []; beq1 = [];
% A1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
A1 = []; b1 = [];
%     step_th23  Rx1      Ry1      Rz1    Px1  Py1  Pz1   Rx2      Ry2      Rz2     Px2  Py2  Pz2   c1       c2       c3       w1  w2  w3
LB1 = [0.05     -1.5708  -1.5708  -1.5708 0    0    0.08  -1.5708  -1.5708  -1.5708 0    0    0.03  1        1        1        0.1  0.1  0.1];
UB1 = [0.25     -0.5     -0.5     -0.5    0.15 0.15 0.15  -0.5     -0.5     -0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
LB2 = [0.05     -0.5     -0.5     -0.5    0    0    0.08  -0.5     -0.5     -0.5    0    0    0.03  1        1        1        0.1  0.1  0.1];
UB2 = [0.05      0.5      0.5     0.5     0.15 0.15 0.15   0.5      0.5      0.5    0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
LB3 = [0.05      0.5      0.5     0.5     0    0    0.08   0.5      0.5     0.5     0    0    0.03  1        1        1        0.1  0.1  0.1];
UB3 = [0.25      1.5708   1.5708  1.5708  0.15 0.15 0.15   1.5708   1.5708  1.5708  0.15 0.15 0.15  13.9999  13.9999  13.9999  1    1    1];
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','iter','StallGenLimit',50,'FunctionTolerance',1e-5,'UseParallel', true, 'UseVectorized', false);
tic;
[x7,fval7,exitflag7,output7] = gamultiobj(FitnessFunction4,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
toc;
% range7 = abs(fval7(:,2)-fval7(:,1));
% [best_range7,index7] = sort(range7);
% final_range7 = best_range7(1:floor(size(x7,1))); 
% final_x7 = x7(index7(1:floor(size(x7,1))),:);
% final_sol7 = horzcat(final_range7,final_x7);
eval_x7 = [fval7 x7(:,2:13)]
save('DELL_opt_structures_results2.mat','x4','fval4','x5','fval5','x6','fval6','x7','fval7');
