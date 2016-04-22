%
% Robert Krug, 2016/04/20
% robert.krug@oru.se
%
% Generates a planar environment which reacts with a force
% f_e=-k_e*dz-c_e*ddz
%
% The environment plane is defnied by the x/y plane of cf_e_
%
classdef Environment
	
	properties (Access = protected)
		cf_e_ %environment frame (represented as a hgtransform)
		vis_scale_
	end
	
	properties  (Access = public)
		h_
		k_e_ %environment stiffness
		c_e_ %environment damping
	end
	
	methods
		function environment = Environment(cf_e,h)
% 			if nargin == 1
% 				environment.h_=figure;
% 			else
% 				environment.h_=h;
% 			end
			scale=1; %define visualization scaling factor
			
			%guesstimate environment stiffness/damping parameters (k=c^2/4 yields critical damping)
			environment.c_e_=150;
			environment.k_e_=environment.c_e_^2/4;
			
			environment.vis_scale_=scale;
			
			phi = linspace(-pi,pi,100);
			x = scale*cos(phi); y = scale*sin(phi); z = zeros(1,100);
			
			%plot plane with normal along z
			h(1)=patch(x,y,z,'c','FaceAlpha',0.25);
			h(2:7)=draw_frame(eye(3),zeros(3,1),'rgb',scale/3);
			
			environment.cf_e_=hgtransform('parent',gca);
			set(h,'parent',environment.cf_e_);
			environment.cf_e_.Matrix=cf_e;
			
			environment.h_=[environment.h_; h(:)];
		end
		
		function fn=getContactNormalForce(environment,p_e,v_e)
			%generate plane H/K representation (a'x+b=0)
			a=environment.cf_e_.Matrix(1:3,3);
			b=-a'*environment.cf_e_.Matrix(1:3,4);
			
			%project end-effector positon & velolcity on the environment normal
			dz=a'*p_e+b;
			ddz=a'*v_e;
			
			if dz > 0
				fn=a*(-dz*environment.k_e_-ddz*environment.c_e_);
			else
				fn=zeros(3,1);
			end	
				
		end
	end
end
