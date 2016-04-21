clear all; close all; clc;
% example showing the usage of the alternative hgtransform based visualizer
% for in-the-loop visualization (much faster alternative then the
% Draw_System function)


SP = model_system_3; %test system
SV = System_Variables(SP);

visualizer=MBSVisualizer(SP,SV); axis equal; %create an instance of the visualizer


SV.q=rand(3,1);
SV=calc_pos(SP,SV); %calc_pos needs to be called before updating the visualization

visualizer.update(SP,SV);% update
