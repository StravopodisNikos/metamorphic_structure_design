function [Meig,Keig] = PlotMassEllipsoid_3D_atTCP(Mb,Jb,gst,p2fig)
%% Blue Elliposid is Mass Ellipsoid
%% Red Ellipsoid is Sicilano Reformulated Ellipsoid
% Inputs: Mass matrix, BodyJacobian, Pointer to figure of ManipulatorFigure
%% load geom3d library
addpath('/home/nikos/matlab_ws/geom3d')
addpath('/home/nikos/matlab_ws/geom3d/geom3d')
addpath('/home/nikos/matlab_ws/geom2d/geom2d')
addpath('/home/nikos/matlab_ws/geom2d/utils')
%   Documentation of fn used: /geom3d/geom3d/drawEllipsoid.m
%
%   drawEllipsoid(ELLI)
%   Displays a 3D ellipsoid on current axis. ELLI is given by:
%   [XC YC ZC A B C PHI THETA PSI],
%   where (XC, YC, ZC) is the ellipsoid center, A, B and C are the half
%   lengths of the ellipsoid main axes, and PHI THETA PSI are Euler angles
%   representing ellipsoid orientation, in degrees.
%   Properties:
%     drawEllipsoid(elli, 'FaceColor', 'r','drawEllipses', true, 'EllipseColor', 'b', 'EllipseWidth', 3);
%     axis equal;
%   drawEllipsoid(..., 'drawEllipses', true)
%   Also displays the main 3D ellipses corresponding to XY, XZ and YZ
%   planes.

%% Mass Ellipsoid
% Mb = P*D*P^(-1)
% eigenvalues are diagonal elements of D, 3D Rotation Matrix is P
[P1,D1] = eig(Mb);

TCPpos = gst(1:3,4)';
AxesLengths = diag(D1)';
EulerAngles = rotm2eul(P1);
DegEulerAngles(1) = rad2deg(EulerAngles(1));
DegEulerAngles(2) = rad2deg(EulerAngles(2));
DegEulerAngles(3) = rad2deg(EulerAngles(3));
MassEllipsoid = horzcat(TCPpos,AxesLengths,DegEulerAngles);
% Plot
figure(p2fig)
drawEllipsoid(MassEllipsoid, 'FaceColor', 'r','drawEllipses', true, 'EllipseColor', 'b', 'EllipseWidth', 3)

Meig1 = diag(D1);
Keig1 = min(Meig1)/max(Meig1);

%% Reformulated Manipulability Ellipsoid
trq_u = [44.7 25.3 25.3]'; % MAX for Dynamixel motors to be used
L = diag(trq_u);
qb = zeros(3,1); % for TCP in Tool Frame 
[J33b_new] = CalculateSquareTCPJacobian_3DoF(Jb,qb);
Q = inv(J33b_new')*Mb'*L^2*Mb*inv(J33b_new);
[P2,D2] = eig(Q);

TCPpos = gst(1:3,4)';
AxesLengths2 = diag(D2./10000)';
EulerAngles2 = rotm2eul(P2);
DegEulerAngles2(1) = rad2deg(EulerAngles2(1));
DegEulerAngles2(2) = rad2deg(EulerAngles2(2));
DegEulerAngles2(3) = rad2deg(EulerAngles2(3));
ReformulatedManipulabilityEllipsoid = horzcat(TCPpos,AxesLengths2,DegEulerAngles2);
% Plot
figure(p2fig)
drawEllipsoid(ReformulatedManipulabilityEllipsoid, 'FaceColor', 'c','drawEllipses', true, 'EllipseColor', 'm', 'EllipseWidth', 3)

Meig2 = diag(D2);
Keig2 = min(Meig2)/max(Meig2);

Meig = horzcat(Meig1,Meig2);
Keig = [Keig1, Keig2];
end