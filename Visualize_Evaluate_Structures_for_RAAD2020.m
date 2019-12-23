% visualize and evaluate structures for raad2020
% In this file the main results presented in paper for RAAD2020 are
% generated. 
% INPUT:
% xi_best_strucure is R,P structure given from ga in MOUL2, as copied in
% file STRUCTURES_COMPARISON.xml
% structureURDFfile is found in folder raad2020/urdf/filename(from MOUL2 ga options)
% structure2check is manually given regarding the structure to be evaluated

clear; clc; close all;

% ONLY STRUCTURES PRESENTED IN PAPER ARE GIVEN
% for the rest, parameters are given accordingly!

% % % S0110110
% 1_1
structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_1_1.urdf';
structure2check = '0110110';
xi_best_S0110110 = [0.0152   -0.7414    1.0098    0.1377    0.1004    0.1073    1.3478    1.0640   -1.3335    0.0330    0.0881    0.1045];
[check_1_1,Nrich_1_1,Nrich_pct_1_1,fbest_1_1,tp_best_1_1,tp_rich_1_1] = FinalStructureEvaluation(xi_best_S0110110,structure2check,structureURDFfile);
% 1_2
structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_1_2.urdf';
structure2check = '0110110';
xi_best_S0110110 = [0.0075   -1.1252    1.0319    0.1444    0.1143    0.1358    1.3836    1.2928   -1.2963    0.0342    0.0879    0.1318];
[check_1_2,Nrich_1_2,Nrich_pct_1_2,fbest_1_2,tp_best_1_2,tp_rich_1_2] = FinalStructureEvaluation(xi_best_S0110110,structure2check,structureURDFfile);
% 1_3
structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_1_3.urdf';
structure2check = '0110110';
xi_best_S0110110 = [-0.3799   -1.5674   -1.0544    0.0335    0.0339    0.1056   -0.2382    1.1017   -1.4540    0.0926    0.0131    0.1490];
[check_1_3,Nrich_1_3,Nrich_pct_1_3,fbest_1_3,tp_best_1_3,tp_rich_1_3] = FinalStructureEvaluation(xi_best_S0110110,structure2check,structureURDFfile);

% S01010
% 2_1
% % structure2check = '01010';
% % xi_best_S01010 = [-0.0099   -1.5655    1.2596    0.1462    0.1496    0.1414   -1.2210   -1.5685    1.0848    0.1113    0.1083    0.1486];
% % structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_2_1.urdf';
% % [check_2_1,Nrich_2_1,Nrich_pct_2_1,fbest_2_1,tp_best_2_1,tp_rich_2_1] = FinalStructureEvaluation(xi_best_S01010,structure2check,structureURDFfile);

% S010110
% % % 3_1
% % structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_3_1.urdf';
% % structure2check = '010110';
% % xi_best_S010110 = [-1.1411   -1.5241   -1.2571    0.0204    0.0235    0.1022    0.0282   -0.0737   -1.0054    0.0315    0.0141    0.1376];
% % [check_3_1,Nrich_3_1,Nrich_pct_3_1,fbest_3_1,tp_best_3_1,tp_rich_3_1] = FinalStructureEvaluation(xi_best_S010110,structure2check,structureURDFfile);
% % % % 3_2
% % structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_3_2.urdf';
% % structure2check = '010110';
% % xi_best_S010110 = [-1.0299   -1.5708   -1.2721    0.0222    0.0590    0.1020   -0.1310   -0.1807   -0.5172    0.0815    0.0246    0.1500];
% % [check_3_2,Nrich_3_2,Nrich_pct_3_2,fbest_3_2,tp_best_3_2,tp_rich_3_2] = FinalStructureEvaluation(xi_best_S010110,structure2check,structureURDFfile);

% S011010
% % % 4_1
% % structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_4_1.urdf';
% % structure2check = '011010';
% % xi_best_S011010 = [0.0036   -0.5746    0.7717    0.1318    0.0435    0.1097   -0.3994   -0.3166   -0.2955    0.0298    0.0564    0.1027];
% % [check_4_1,Nrich_4_1,Nrich_pct_4_1,fbest_4_1,tp_best_4_1,tp_rich_4_1] = FinalStructureEvaluation(xi_best_S011010,structure2check,structureURDFfile)
% % % 4_2
% % structureURDFfile = '/home/nikos/matlab_ws/modular_dynamixel/raad2020/urdf/final/final_struct_4_1.urdf';
% % structure2check = '011010';
% % xi_best_S011010 = [0.0036   -0.5746    0.7717    0.1318    0.0435    0.1097   -0.3994   -0.3166   -0.2955    0.0298    0.0564    0.1027];
% % [check_4_2,Nrich_4_2,Nrich_pct_4_2,fbest_4_2,tp_best_4_2,tp_rich_4_2] = FinalStructureEvaluation(xi_best_S011010,structure2check,structureURDFfile)