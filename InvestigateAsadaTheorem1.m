clc;
clear;
close all;
%% INFO
% Builds 2 DoF Metamorphic structures S010 and S0110
% Investigates Dynamic Decoupling conditions:
% 1. Theorem1 by Asada in 
% 2. DCI "exhaustive search" in S010 & S0110
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
addpath('/home/nikos/matlab_ws/geom3d')
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')

%% load zero data for STRUCTURAL BLOCKS
% data are obtained by .m files:
% modular_dynamixel/structural_synthesis/kinematic_verification_C01.m for SB01
% modular_dynamixel/structural_synthesis/kinematic_verification_C011.m for SB011
load('SB10_zero_data.mat');
pi_10 = pi(:,2:4); % j-i1-k(!)
wi_10 = wi(:,2:3); % j-i1
g0_10 = g0;
gsc_10 = g0(:,:,9); % g_s_lk0
g_lk_TOOL_10 = g0(:,:,10);
xi_10 = xi;
load('SB110_zero_data.mat');
pi_110 = pi(:,2:5); % j1-j2-i1-k(!)
wi_110 = wi(:,2:4); % j1-j2-i1
g0_110 = g0;
gsc_110 = g0(:,:,2); % g_s_lk0
g_lk_TOOL_110 = g0(:,:,14);
xi_110 = xi;
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
s2 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7);
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
s3 = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7);

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

t10 = [0 0]';
n = 2; % Structural Block number
R010 = [0 0 0]; 
P010 = [0.1 0 0];
[S010] = Build_SB10_forGA(s1,s2,R010,P010,n,s010_fig,[t0; t10],x1);
a010 = 3; % column of second active joint in final Js

%% S0110
s0110_name = '/home/nikos/matlab_ws/modular_dynamixel/structure_synthesis/S0110_1.urdf';
ShowAnatomyFigure(s0110_name,s0110_fig)
t110 = [1.5708 0.7854 0]';
n = 2; % Structural Block number
R0110 = [0 0 0]; % normally ga gives them
P0110 = [0.1 0 0]; % normally ga gives them
[S0110] = Build_SB110_forGA(s1,s3,R0110,P0110,n,s0110_fig,[t0; t110],x1);
a0110 = 4; % column of second active joint in final Js
