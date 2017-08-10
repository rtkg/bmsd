% uses function normalize_angles.m

clear all; close all; clc
% system definition
SP = model_UR10();
SV = System_Variables(SP);

SV.q=[-0.0216056    0.132321  0.00251184  0.00979541    0.011501 -0.00725083 ]';
SV.dq=[-0.0458164   0.0278134  0.00061929  0.00264415  0.00139838 0.000125537]';

% updates positions & velocities of links
SV = calc_pos(SP,SV); 
SV = calc_vel(SP,SV); 

%forward kinematics & end-effector Jacobian
[pE,RE] = fk_e(SP,SV,SP.bN,SP.bP,SP.bR);
Je = calc_Je(SP,SV,SP.bN,SP.bP); 

[dJe,joints] = calc_dJe(SP,SV,SP.bN,SP.bP);
%Jacobian derivative

Draw_System(SP,SV,SP.bN,SP.bP,1:SP.n);
grid on; rotate3d on; axis equal;
%%%EOF