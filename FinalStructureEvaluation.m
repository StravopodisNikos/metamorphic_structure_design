function [check,Nrich,Nrich_pct,fbest,tp_best,tp_rich] = FinalStructureEvaluation(xi_best,structure2check,structureURDFfile)
% Evaluates structures extracted from MOUL2 optimization files:
% 1. Investigate3DoF_MassMatrixRAAD2020.m
% 2. Investigate3DoF_MassMatrixRAAD2020_NoPseudo.m
% Takes best ga structure and evaluates kinematics and TCP Mass Ellipsoid
% Condition number is plotted in C-space of reference anatomy only!
% --- Rules of the above files are considered executed! ---
% After reference anatomy of structure investigated is extracted
% the PC-Space is examined. Nrich is the absolute number of anatomies that
% present fanat > fref. f = (maxDCI+minDCI)/2 @ whole C-space

% Call for S0110110 example:
% structure2check = '0110110'
% structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/raad2020_ga500_1200_S0110110.urdf'
% xi_best = [-1.4786   -0.5168    1.5020    0.0298    0.0011    0.0926 -0.5318   -1.5420   -1.1597    0.0042    0.0013    0.0314]

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

%% Get best chromosome values
R_1 = xi_best(1:3);
P_1 = xi_best(4:6);
R_2 = xi_best(7:9);
P_2 = xi_best(10:12);
wi_best = [0.45 0.45 0.1]';
step_a23 = deg2rad(15);
%% Dependng on structure string entered, initialize pseudoangles to ref
k = 1; % counter of pseudojoints in structure
for i = 1:length(structure2check)
    if structure2check(i) == '1'
        tp(k) = 0;
        k = k+1;
    end
end

%% Kinematics Evaluation
switch structure2check
    case '0110110'
%% =========================     S0110110     ===========================
        %% A. First kinematics are calculated through MATLAB solver
        % robot urdf is built from /structure_synthesis/S0110110.xacro
        % given manually the xi values, FOR ZERO PSEUDO ANGLES!!!!
        [robotS0110110] = importrobot(structureURDFfile);  
        robotS0110110.DataFormat = 'column';
        robotS0110110.Gravity = [0 0 -9.80665];
        robotS0110110_figure = figure;
        %% Show reference anatomy robot
        figure(robotS0110110_figure);
        config = [0 0 0]';
        show(robotS0110110,config,'PreservePlot',false);
        hold on;
        axis auto;
        box on;
        %% B. Here my kinematics solver is used (the one used in ga)
        n = 2; % Structural Block number
        [S2] = Build_SB110_forGMinMaxDCI(s1,s3,R_1,P_1,n);
        xi_ai(:,1) = s1.xi;
        xi_ai(:,2) = S2.xi(:,3);
        xi_pi(:,1) = S2.xi(:,1);
        xi_pi(:,2) = S2.xi(:,2);
        %gst0 = S2.Cg;
        gsCoM10 = S2.g_s_CoM0;

        n = 3; % Structural Block number
        [S3] = Build_SB110_forGMinMaxDCI(S2,s3,R_2,P_2,n);
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

        Mbi(:,:,1) = s3.Mbi(:,:,1);
        Mbi(:,:,2) = s3.Mbi(:,:,1);
        Mbi(:,:,3) = s3.Mbi(:,:,2);
        %% Sets current anatomy pseudo exponential
        P1 = twistexp(xi_pi(:,1),tp(1))*twistexp(xi_pi(:,2),tp(2));
        P2 = twistexp(xi_pi(:,3),tp(3))*twistexp(xi_pi(:,4),tp(4));
        Pi(:,:,1) = P1;
        Pi(:,:,2) = P2;
        
        %% Evaluation is executed at random configuration!
        qrand = config;
        
        % 1. FKP
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),qrand(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),qrand(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),qrand(3));
        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0;
        % 2. Js,Jb
        Js(:,1) = xi_ai(:,1);
        Js(:,2) = ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai(:,2);
        Js(:,3) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai(:,3);
        figure(robotS0110110_figure)
        plot_xi2 = drawtwist(Js(:,2)); hold on;
        plot_xi3 = drawtwist(Js(:,3)); hold on;
        % Jbody
        Jb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,1);
        Jb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,2);
        Jb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai(:,3);
        
        % 3. Mb
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        
        % 4. Ellipsoid
        [Meig,Keig] = PlotMassEllipsoid_3D_atTCP(Mb,Jb,gst,robotS0110110_figure);

        % 5. Calculate fref for structures's anatomies richness
        fref = ActiveCspace_for_AnatomyRichnessCalculation(step_a23,wi_best,xi_ai,Pi,gsCoMi0,Mbi);
        
        disp('Structure S0110110 was evaluated.')
        check = 'ok';
%% =========================     S0110110     ===========================

    case '01010'
%% =========================      S01010      ===========================
        %% A. First kinematics are calculated through MATLAB solver
        % robot urdf is built from /structure_synthesis/S0110110.xacro
        % given manually the xi values, FOR ZERO PSEUDO ANGLES!!!!
        [robotS01010] = importrobot(structureURDFfile);  
        robotS01010.DataFormat = 'column';
        robotS01010.Gravity = [0 0 -9.80665];
        robotS01010_figure = figure;
        %% Show reference anatomy robot
        figure(robotS01010_figure);
        config = [0 0 0]';
        show(robotS01010,config,'PreservePlot',false);
        hold on;
        axis auto;
        box on;
        %% B. Here my kinematics solver is used (the one used in ga)
        n = 2; % Structural Block number
        [S2] = Build_SB10_forGMinMaxDCI(s1,s2,R_1,P_1,n);
        xi_ai(:,1) = s1.xi;
        xi_ai(:,2) = S2.xi(:,2);
        xi_pi(:,1) = S2.xi(:,1);
        %gst0 = S2.Cg;
        gsCoM10 = S2.g_s_CoM0;

        n = 3; % Structural Block number
        [S3] = Build_SB10_forGMinMaxDCI(S2,s2,R_2,P_2,n);
        %xi_ai(:,1) = s1.xi;
        xi_ai(:,3) = S3.xi(:,2);
        xi_pi(:,2) = S3.xi(:,1);
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
        P1 = twistexp(xi_pi(:,1),tp(1));
        P2 = twistexp(xi_pi(:,2),tp(2));
        Pi(:,:,1) = P1;
        Pi(:,:,2) = P2;
        %% Evaluation is executed at random configuration!
        qrand = config;
        
        % 1. FKP
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),qrand(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),qrand(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),qrand(3));
        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0;
        % 2. Js,Jb
        Js(:,1) = xi_ai(:,1);
        Js(:,2) = ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai(:,2);
        Js(:,3) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai(:,3);
        figure(robotS01010_figure)
        plot_xi2 = drawtwist(Js(:,2)); hold on;
        plot_xi3 = drawtwist(Js(:,3)); hold on;
        % Jbody
        Jb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,1);
        Jb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,2);
        Jb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai(:,3);
        
        % 3. Mb
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        
        % 4. Ellipsoid
        [Meig,Keig] = PlotMassEllipsoid_3D_atTCP(Mb,Jb,gst,robotS01010_figure);

        % 5. Calculate fref for structures's anatomies richness
        fref = ActiveCspace_for_AnatomyRichnessCalculation(step_a23,wi_best,xi_ai,Pi,gsCoMi0,Mbi);
        
        disp('Structure S01010 was evaluated.')
        check = 'ok';
%% =========================      S01010      ===========================

    case '010110'
%% =========================      S010110     ===========================
        %% A. First kinematics are calculated through MATLAB solver
        % robot urdf is built from /structure_synthesis/S0110110.xacro
        % given manually the xi values, FOR ZERO PSEUDO ANGLES!!!!
        [robotS010110] = importrobot(structureURDFfile);  
        robotS010110.DataFormat = 'column';
        robotS010110.Gravity = [0 0 -9.80665];
        robotS010110_figure = figure;
        %% Show reference anatomy robot
        figure(robotS010110_figure);
        config = [0 0 0]';
        show(robotS010110,config,'PreservePlot',false);
        hold on;
        axis auto;
        box on;
        %% B. Here my kinematics solver is used (the one used in ga)
        n = 2; % Structural Block number
        [S2] = Build_SB10_forGMinMaxDCI(s1,s2,R_1,P_1,n);
        xi_ai(:,1) = s1.xi;
        xi_ai(:,2) = S2.xi(:,2);
        xi_pi(:,1) = S2.xi(:,1);
        %gst0 = S2.Cg;
        gsCoM10 = S2.g_s_CoM0;
        
        n = 3; % Structural Block number
        [S3] = Build_SB110_forGMinMaxDCI(S2,s3,R_2,P_2,n);
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
        Mbi(:,:,2) = s3.Mbi(:,:,1);
        Mbi(:,:,3) = s3.Mbi(:,:,2);   
        
        %% Sets current anatomy pseudo exponential
        P1 = twistexp(xi_pi(:,1),tp(1));
        P2 = twistexp(xi_pi(:,2),tp(2))*twistexp(xi_pi(:,3),tp(3));
        Pi(:,:,1) = P1;
        Pi(:,:,2) = P2;
        
        %% Evaluation is executed at random configuration!
        qrand = config;
        
        % 1. FKP
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),qrand(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),qrand(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),qrand(3));
        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0;
        % 2. Js,Jb
        Js(:,1) = xi_ai(:,1);
        Js(:,2) = ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai(:,2);
        Js(:,3) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai(:,3);
        figure(robotS010110_figure)
        plot_xi2 = drawtwist(Js(:,2)); hold on;
        plot_xi3 = drawtwist(Js(:,3)); hold on;
        % Jbody
        Jb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,1);
        Jb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,2);
        Jb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai(:,3);
        
        % 3. Mb
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        
        % 4. Ellipsoid
        [Meig,Keig] = PlotMassEllipsoid_3D_atTCP(Mb,Jb,gst,robotS010110_figure);
        
        % 5. Calculate fref for structures's anatomies richness
        fref = ActiveCspace_for_AnatomyRichnessCalculation(step_a23,wi_best,xi_ai,Pi,gsCoMi0,Mbi);

        disp('Structure S010110 was evaluated.')
        check = 'ok';
%% =========================      S010110     ===========================

    case '011010'
%% =========================      S011010     ===========================
        %% A. First kinematics are calculated through MATLAB solver
        % robot urdf is built from /structure_synthesis/S0110110.xacro
        % given manually the xi values, FOR ZERO PSEUDO ANGLES!!!!
        [robotS011010] = importrobot(structureURDFfile);  
        robotS011010.DataFormat = 'column';
        robotS011010.Gravity = [0 0 -9.80665];
        robotS011010_figure = figure;
        %% Show reference anatomy robot
        figure(robotS011010_figure);
        config = [0 0 0]';
        show(robotS011010,config,'PreservePlot',false);
        hold on;
        axis auto;
        box on;
        
        n = 2; % Structural Block number
        [S2] = Build_SB110_forGMinMaxDCI(s1,s3,R_1,P_1,n);
        xi_ai(:,1) = s1.xi;
        xi_ai(:,2) = S2.xi(:,3);
        xi_pi(:,1) = S2.xi(:,1);
        xi_pi(:,2) = S2.xi(:,2);
        %gst0 = S2.Cg;
        gsCoM10 = S2.g_s_CoM0;

        n = 3; % Structural Block number
        [S3] = Build_SB10_forGMinMaxDCI(S2,s2,R_2,P_2,n);
        %xi_ai(:,1) = s1.xi;
        xi_ai(:,3) = S3.xi(:,2);
        xi_pi(:,3) = S3.xi(:,1);
        gst0 = S3.Cg;
        gsCoM20 = S3.g_s_CoM0;

        gsCoMi0(:,:,1) = gsCoM10(:,:,1);
        gsCoMi0(:,:,2) = gsCoM20(:,:,1);
        % gsCoMi0(:,:,3) = gsCoM20(:,:,2);
        gsCoMi0(:,:,3) = S3.Cg; % approximation

        Mbi(:,:,1) = s3.Mbi(:,:,1);
        Mbi(:,:,2) = s2.Mbi(:,:,1);
        Mbi(:,:,3) = s2.Mbi(:,:,2);
        %% Sets current anatomy pseudo exponential
        P1 = twistexp(xi_pi(:,1),tp(1))*twistexp(xi_pi(:,2),tp(2));
        P2 = twistexp(xi_pi(:,3),tp(3));
        Pi(:,:,1) = P1;
        Pi(:,:,2) = P2;
        
        %% Evaluation is executed at random configuration!
        qrand = config;
        
        % 1. FKP
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),qrand(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),qrand(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),qrand(3));
        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0;
        % 2. Js,Jb
        Js(:,1) = xi_ai(:,1);
        Js(:,2) = ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai(:,2);
        Js(:,3) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai(:,3);
        figure(robotS011010_figure)
        plot_xi2 = drawtwist(Js(:,2)); hold on;
        plot_xi3 = drawtwist(Js(:,3)); hold on;
        % Jbody
        Jb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,1);
        Jb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,2);
        Jb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai(:,3);
        
        % 3. Mb
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        
        % 4. Ellipsoid
        [Meig,Keig] = PlotMassEllipsoid_3D_atTCP(Mb,Jb,gst,robotS011010_figure);
        
        % 5. Calculate fref for structures's anatomies richness
        fref = ActiveCspace_for_AnatomyRichnessCalculation(step_a23,wi_best,xi_ai,Pi,gsCoMi0,Mbi);
        
        disp('Structure S011010 was evaluated.')
        check = 'ok';        
%% =========================      S011010     =========================== 

    otherwise
        disp('Structure given is not accepted.')
        check = 'error';  
end

%% Anatomies richness of evaluated structure
[Nrich,Nrich_pct,fbest,tp_best,tp_rich] = EvaluateAnatomiesRichnessForBestInertiaStructure(fref,step_a23,wi_best,xi_ai,xi_pi,structure2check,gsCoMi0,Mbi);
end