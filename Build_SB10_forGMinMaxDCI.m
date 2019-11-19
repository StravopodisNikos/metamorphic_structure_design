function [Si] = Build_SB10_forGMinMaxDCI(Sim1,Si0,R,P,SBn)
% Calculates xi_ai,xi_pi and zero tfs for Structural Block S01
% Works ONLY for 2 DoF for /raad2020/InvestigateAsadaTheorem1.m
% INPUT: 
% OUTPUT:

%% Synthetic Joint is parameterized to exponential
% Rotation matrix due to synthetic joint
Rx01 = R(1); Ry01 = R(2); Rz01 = R(3);
Px01 = P(1); Py01 = P(2); Pz01 = P(3);
R_lk = rotz(Rz01)*roty(Ry01)*rotx(Rx01);
[wmegaR thetaR] = rotparam(R_lk); % this is SO(3) element-skew matrix and angle
exp_wmegaR = skewexp(wmegaR,thetaR);

%% Check if SB10 is 2nd or 3rd block
%% Regarding the position only the initial gs changes
if SBn==2
        g_s_lk_1_0 = Sim1.Cg;
        xi_lk = createtwist(wmegaR,[g_s_lk_1_0(1,4)+Px01 g_s_lk_1_0(2,4)+Py01 g_s_lk_1_0(3,4)+Pz01]'); %ξk
        g_slk = [exp_wmegaR [g_s_lk_1_0(1,4)+Px01 g_s_lk_1_0(2,4)+Py01 g_s_lk_1_0(3,4)+Pz01]'; 0 0 0 1  ];
        exp_lk = twistexp(xi_lk,thetaR);

  elseif SBn ==3
        g_slk_loc = [exp_wmegaR [Px01 Py01 Pz01]'; 0 0 0 1  ];
%         poutana = eye(4); poutana(1,4) = -0.091;
        g_slk = Sim1.Rk*g_slk_loc;
%         [xi_lk theta_xi_lk] = homtotwist(g_slk);
        exp_wmegaR = g_slk(1:3,1:3);
        [wmega_lk theta_lk] = rotparam(exp_wmegaR);
        xi_lk = createtwist(wmega_lk,[g_slk(1,4) g_slk(2,4) g_slk(3,4)]'); %ξk
end

% The twists of SB10 are re-calculated due to Synthetic Joint
wjn = exp_wmegaR*Si0.wi(:,1); g_sP1b = g_slk*Si0.g0(:,:,8);
xjn = createtwist(wjn,g_sP1b(1:3,4)); %ξj'
wi1n = exp_wmegaR*Si0.wi(:,2); g_sf1 = g_slk*Si0.g0(:,:,8)*Si0.g0(:,:,5);
xi1n = createtwist(wi1n,g_sf1(1:3,4)); %ξi+1'
Xi_for_struct(:,1) = xjn;
Xi_for_struct(:,2) = xi1n;

gn(:,:,1) = g_slk*Si0.g0(:,:,8); % g_s_lj0n = g_s_lk*g_lk_lj0
gn(:,:,2) = g_slk*Si0.g0(:,:,8)*Si0.g0(:,:,5); % g_s_li10n
% gn(:,:,3) = inv(Sim1.g0)*gn(:,:,1); % g_li_lj0 
gn(:,:,3) = inv(Sim1.gsli1)*gn(:,:,1); % g_li_lj0
gn(:,:,4) = inv(gn(:,:,1))*gn(:,:,2); % g_lj_li10
% gn(:,:,5) = inv(Sim1.g0)*gn(:,:,2); % g_li_li10
gn(:,:,5) = inv(Sim1.gsli1)*gn(:,:,2); % g_li_li10
gn(:,:,6) = g_slk*Si0.g0(:,:,11); % g_s_CoMi0 = g_s_lk*g_lk_CoMi0
gn(:,:,7) = g_slk*Si0.g0(:,:,12); % g_s_CoM_TOOL = g_s_lk*g_lk_CoM_TOOL

NEW_t0_FRAME = g_slk*Si0.Cg;

% xi_j_new = inv(ad(Sim1.gsli1))*xjn;
% xi_i1_new = inv(ad(Sim1.gsli1))*xi1n;
% xj_i1_new = inv(ad(gn(:,:,1)))*xi1n;

% exp1 = twistexp(Xi1,ti(1));
% expj = twistexp(xjn, ti(2));
% expi1 = twistexp(xi1n, ti(3));
% exp_for_struct(:,:,1) = expj;
% exp_for_struct(:,:,2) = expi1;

% if SBn==2
%     gnj = exp1*expj*gn(:,:,1); % g_s_lj
%     gni1 = exp1*expj*expi1*gn(:,:,2); % g_s_li1
%     gnCoMi1 = exp1*expj*expi1*gn(:,:,6); % g_s_CoMi1
%     gnCoMtool = exp1*expj*expi1*gn(:,:,7); % g_s_CoM_TOOL
%     gnst = exp1*expj*expi1*NEW_t0_FRAME;
% elseif SBn==3
%     gnj = expj*gn(:,:,1); % g_s_lj
%     gni1 = expj*expi1*gn(:,:,2); % g_s_li1
%     gnCoMi1 = expj*expi1*gn(:,:,6); % g_s_CoMi1
%     gnCoMtool = expj*expi1*gn(:,:,7); % g_s_CoM_TOOL
%     gnst = expj*expi1*NEW_t0_FRAME;   
% end

% gsn(:,:,1) = gnj;
% gsn(:,:,2) = gni1;
% gsn(:,:,3) = gnst;

g_s_CoM0(:,:,1) = gn(:,:,6);
g_s_CoM0(:,:,2) = NEW_t0_FRAME;

% g_s_CoM(:,:,1) = gnCoMi1;
% g_s_CoM(:,:,2) = gnCoMtool;
% g_li_li1 = twistexp(xi_j_new,ti(2))*gn(:,:,3)*twistexp(xj_i1_new,ti(3))*gn(:,:,4); % for synthetic+metamorphic
% %% Relative twists
% [xi_i1_abs th_i_i1_abs] = homtotwist(g_li_li1) % the absolute transformation twist- READS structure change
% g = g_li_li1*inv(gn(:,:,5));
% [xi_i1_rel th_i_i1_rel] = homtotwist(g) % the relative transformation twist - READS pseudojoint change

% if SBn==2 
%     Js(:,1) = ad(exp1)*xi_lk;
%     Js(:,2) = ad(exp1)*xjn;
%     Js(:,3) = ad(exp1*expj)*xi1n;
% elseif SBn==3
%     Js(:,1) = ad(eye(4))*xi_lk;
%     Js(:,2) = ad(eye(4))*xjn;
%     Js(:,3) = ad(expj)*xi1n;
% end

% figure(p2fig); % for visual evaluation
% xk_graph = drawtwist(Js(:,1)); hold on;
% xj_graph = drawtwist(Js(:,2)); hold on;
% xi1_graph = drawtwist(Js(:,3)); hold on;

%% Build OUTPUT struct
f1 = 'Cg'; v1 = NEW_t0_FRAME; %gst0
f2 = 'xi'; v2 = Xi_for_struct;
f3 = 'g_s_CoM0'; v3  = g_s_CoM0;
Si = struct(f1,v1,f2,v2,f3,v3);
end