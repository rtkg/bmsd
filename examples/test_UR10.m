% uses function normalize_angles.m

clear; close; clc
% system definition
SP = model_UR10();
SV = System_Variables(SP);

SV.q = ones(6,1)*0.5;

% updates positions of links
SV = calc_pos(SP,SV); 
SV = calc_pos(SP,SV);
[pE,RE] = fk_e(SP,SV,SP.bN,SP.bP,SP.bR);

Draw_System(SP,SV,SP.bN,SP.bP,1:SP.n);
grid on; rotate3d on; axis equal;

%%%EOF