function J = jacobian()
% Finds the Jacobian of the Manipulator (from base frame to end-effector
% given the design parameters)

%% Calculating Transformation Matrix
syms theta1 theta2 theta3

load('parameters'); %load design parameters into workspace

A01=compute_dh_matrix(a1,alpha1,d1,theta1); %homogenous DH matrix from base frame '0' to frame '1'
A12=compute_dh_matrix(a2,alpha2,d2,theta2);
A23=compute_dh_matrix(a3,alpha3,d3,theta3);
A34=[r11 r12 r13 ex;r21 r22 r23 ey;r31 r32 r33 ez; 0 0 0 1]; %from '3' to end effector frame
%calculating the total transformation matrices by using series of transformations
T01=A01;  %from '0' to '1'
T02=A01*A12;
T03=A01*A12*A23;
T04=A01*A12*A23*A34; %from base frame '0' to end effector frame '4'

%% Calculating the 'Z' and 'P' components of each transformation matrix (reference: edX course by UPenn)

Z1=T01(1:3,3);
Z2=T02(1:3,3);
Z3=T03(1:3,3);
Z4=T04(1:3,3);

P1=T01(1:3,4);
P2=T02(1:3,4);
P3=T03(1:3,4);
P4=T04(1:3,4);
 
Z0=[0;0;1]; %initialization of Z and P
P0=[0;0;0]; 

%% Calculating Jacobian using crossproduct

J1=[cross(Z0, P4-P0);Z0]; %Jacobians
J2=[cross(Z1, P4-P1);Z1];
J3=[cross(Z2, P4-P2);Z2];
J4=[cross(Z3, P4-P3);Z3];

J=[J1 J2 J3 J4];


end

