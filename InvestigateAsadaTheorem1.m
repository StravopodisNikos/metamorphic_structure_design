clc;
clear;
close all;
%% INFO
% Builds 2 DoF Metamorphic structures S010 and S0110
% Investigates Dynamic Decoupling conditions:
% 1. Theorem1 by Asada in 
% 2. DCI "ga search" in S010 & S0110
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

%% Figures
% structures urdf are built from: /home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/kinematic_verification_01.xacro
%                                 /home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/kinematic_verification_011.xacro
% .xacro {change params} - > .urdf
% URDF MODELS USED:
% S010  -> /structure_synthesis/S010_i.urdf
% S0110 -> /structure_synthesis/S0110_i.urdf

s010_fig = figure;
s0110_fig = figure;


% SBs are determined by me at this stage-manual investiagtion for 2 DoF
% structure(1,:) = 'xxSB0';
% structure(2,:) = 'xSB10'; OR structure(2,:) = 'SB110';

%% S010 
s010_name = '/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S010_1.urdf';
ShowAnatomyFigure(s010_name,s010_fig)

% % % S010_MinMaxDCI1_name = '/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S010_MinMaxDCI1.urdf';
% % % ShowAnatomyFigure(S010_MinMaxDCI1_name,s010_fig)
% % %% Here call ga's
% % FitnessFunction1 = @(x)GlobalMinMaxDCI_S010(x,s1,s2);
% % nvars1 = 11;
% % Aeq1 = []; beq1 = [];
% % A1 = [0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
% % %     step_th2  Rx        Ry      Rz      Px   Py   Pz   ci      w11  w22  w12
% % LB1 = [0.05     0        0       0       0.1  0.1  0.1  1       0.1  0.1  0.1];
% % UB1 = [0.1      1.5708   1.5708  1.5708  0.15 0.15 0.15 37.9999 1    1    1  ];
% % % IntCon1 = 8;
% % % for w1=w2=0 use 'FunctionTolerance',1e-13, for w1=w2=1000 use 'FunctionTolerance',1e-5
% % options = optimoptions('ga','Generations',300,'PopulationSize',800,'Display','iter','FunctionTolerance',1e-5,'StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% % tic
% % [x1,fval1,exitflag1,output1] = ga(FitnessFunction1,nvars1,[],[],Aeq1,beq1,LB1,UB1,[],[],options);
% % toc
% % % 2. Rx,Rz=[-pi/2,0] Ry=[0,pi/2] ci:13 for deg2rad(15) ci:37 for deg2rad(5)
% % LB2 = [0.05   -1.5708   -1.5708     -1.5708  -0.15  -0.15  -0.15  1       0.1 0.1 0.1];
% % UB2 = [0.1     0        0           0        -0.1   -0.1   -0.1   37.9999 1   1   1  ];
% % options = optimoptions('ga','Generations',300,'PopulationSize',800,'Display','iter','FunctionTolerance',1e-5,'StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
% % tic
% % [x2,fval2,exitflag2,output2] = ga(FitnessFunction1,nvars1,[],[],Aeq1,beq1,LB2,UB2,[],[],options);
% % toc
% % %% optimization results are saved in /raad2020/InvestigateAsadaTheorem1_logfile
% % MinMaxDCIevaluation_S010(x1,s1,s2)
% % MinMaxDCIevaluation_S010(x2,s1,s2)

%% S0110
s0110_name = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/S0110.urdf';
ShowAnatomyFigure(s0110_name,s0110_fig)

FitnessFunction1 = @(x)GlobalMinMaxDCI_S0110(x,s1,s3);
nvars1 = 12;
IntCon1 = [8,9];
Aeq1 = []; beq1 = [];
A1 = [0 0 0 0 0 0 0 0 0 1 1 1; 0 0 0 0 0 0 0 0 0 -1 -1 -1]; b1 = [1; -1];
%     step_th2  Rx        Ry      Rz      Px   Py   Pz  c1  c2  w11  w22  w12
LB1 = [0.05     -0.5     -0.5   -0.5      0.1  0.1  0.1  1   1   0.1  0.1  0.1];
UB1 = [0.1      1.5708   1.5708  1.5708  0.15 0.15 0.15 37  37    1    1    1  ];
options = optimoptions('ga','Generations',10,'PopulationSize',100,'Display','iter','FunctionTolerance',1e-9,'StallGenLimit',50,'UseParallel', true, 'UseVectorized', false);
tic
[x1,fval1,exitflag1,output1] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options);
toc
% x1 = 0.0780    1.2738   -0.4713   -0.4975    0.1001    0.1000    0.1143   14.0000   29.0000    0.2383    0.2927    0.4688
% fval1 = 5.6182e-04
% [tp_S0110] = MinMaxDCIevaluation_S0110(x1,s1,s3)