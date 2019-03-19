function T = force_to_torque(F,t1,t2,t3)
% -This function calculates the joint torques that would be required 
% given the external force and moments on end effector at that
% configuration

% -NOTE: This is a STATIC analysis so it does not consider 
% dynamics properties like manipulator inertias

% -F in [Fx;Fy;Fz;Mx;My;Mz] form in Newton and N.m in base frame (refer
% fig: for coordinate systems
% -T (torque) output of motor  will be in [T1;T2;T3] form since this is 3DOF in N.m
% -Configuration is specified by t1 t2 t3 in RADIANS.

J=jacobian();%calculate Jacobian for end-effector frame
T=J'*F;
syms theta1 theta2 theta3
T=double(subs(T,[theta1 theta2 theta3],[t1,t2,t3]));
end

