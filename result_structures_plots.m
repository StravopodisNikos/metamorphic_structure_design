clear;
clc
close all;
%% Here result structures and anatomies for RAAD2020 paper

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

%% Here plots of Mass matrix elements of 2 DoF S010,S0110 from
%% InvestigateAsadaTheorem1_moul2.m ga optimization

%% Initial SB structs
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


%% Next results presented are extracted from logfiles:
% /raad2020/InvestigateAsadaTheorem1_S010_logfile.txt
% /raad2020/InvestigateAsadaTheorem1_S0110_logfile.txt
s010_fig_1 = figure;
s010_fig_2 = figure;
s010_fig_3 = figure;
s0110_fig_1 = figure;
s0110_fig_2 = figure;
s0110_fig_3 = figure;
%% S010
best_S010(1,:) = [0.0957 -0.5013 -0.5001 -0.5054 0.0070 0.0001 0.1281 28  0.3445 0.2951 0.3596];
best_S010(2,:) = [0.1098 -0.1450 -0.0109 -0.0738 0.0016 0.0069  0.1202 19 0.3368 0.3407 0.3223];
best_S010(3,:) = [0.1327  0.5097  0.5064  0.5140  0.0001 0.0119 0.1072 9   0.2403 0.2731 0.4869];
best_S010_name_1 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_6.urdf';
best_S010_name_2 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_7.urdf';
best_S010_name_3 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_8.urdf';
ShowAnatomyFigure(best_S010_name_1,s010_fig_1)
ShowAnatomyFigure(best_S010_name_2,s010_fig_2)
ShowAnatomyFigure(best_S010_name_3,s010_fig_3)

[tp_S010_1] = MinMaxDCIevaluation_S010(best_S010(1,:),s1,s2);
[tp_S010_2] = MinMaxDCIevaluation_S010(best_S010(2,:),s1,s2);
[tp_S010_3] = MinMaxDCIevaluation_S010(best_S010(3,:),s1,s2);

%% S0110
best_S0110(1,:) = [0.1004 -0.5000 -0.5009 -0.5077 0.0102 0.0001 0.1269 28   34   0.2891 0.3464 0.3652];
best_S0110(2,:) = [0.1089 -0.0763 -0.0243 -0.0296  0.0031 0.0018  0.1140 20  21  0.3328 0.3435 0.3231];
best_S0110(3,:) = [0.1342  1.5643  0.5016  0.5037  0.0002 0.0497 0.1157 5   1    0.2582  0.2494 0.4923];
best_S0110_name_1 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_4_1.urdf';
best_S0110_name_2 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_5_1.urdf';
best_S0110_name_3 = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/AsTh1_6_1.urdf';
ShowAnatomyFigure(best_S0110_name_1,s0110_fig_1)
ShowAnatomyFigure(best_S0110_name_2,s0110_fig_2)
ShowAnatomyFigure(best_S0110_name_3,s0110_fig_3)

[tp_S0110_1] = MinMaxDCIevaluation_S0110(best_S0110(1,:),s1,s3);
[tp_S0110_2] = MinMaxDCIevaluation_S0110(best_S0110(2,:),s1,s3);
[tp_S0110_3] = MinMaxDCIevaluation_S0110(best_S0110(3,:),s1,s3);