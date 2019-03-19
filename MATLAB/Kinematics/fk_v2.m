                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   function a = fk_v2(t1,t2,t3)
%forward kinematics of the manipulator
%   takes input t1,t2,t3 (joint angles) in RADIANS and gives the x,y,z coordinates of EEE
%   in METRES

%% Load design parameters
load('parameters');
theta1=t1; 
theta2=t2;
theta3=t3;

%% Calculate Transformation matrix
A01=compute_dh_matrix(a1,alpha1,d1,theta1);
A12=compute_dh_matrix(a2,alpha2,d2,theta2);
A23=compute_dh_matrix(a3,alpha3,d3,theta3);
A34=[r11 r12 r13 ex;r21 r22 r23 ey;r31 r32 r33 ez; 0 0 0 1]; %from 3 to EE
T04=A01*A12*A23*A34;
 
%% Calculate final position of ee in base frame
A=T04;
x=A(1,4);y=A(2,4);z=A(3,4);
a.x=x;
a.y=y;
a.z=z;
end

