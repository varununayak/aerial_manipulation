function a = ik_two_sol(x, y, z)
% -Inverse kinematics for two solutions of inverse kinematics - Elbow up and
% Elbow down solution for -90<theta1<90 degrees
% -Answer is stored in 2X3 matrix 'a' in radians and a_deg in degrees
% -Assumes planar decoupled analysis (using spherical coordinates)
% -The Elbow-up configuration is used in implementation due to mechanical
% stability 

%% Link Design Parameters
load('parameters');

%% Initialization and solving for theta1 (base yaw joint)

a=inf(2,3); %initializing answer variable, infinite or NaN will remain if there's no solution 
phi=Inf(1,2); %intermediate variable
zeta=Inf(1,2); %intermediate variable

theta1=atan2(y,x); %solved for theta1 (one possible solution only)
a(:,1)=theta1; %storing the answer

%% Moving from 3D to 2D;

%transformation from base frame to first frame which is used as origin in
%2-D analysis

X=[cos(theta1),sin(theta1),0,-dl1;...
        0,0,1,-l1;...
        sin(theta1), -cos(theta1),0,0;...
        0,0,0,1]*[x;y;z;1];
 
    
%% x and y definitions change from this point (coming to 2-D (refer fig:)
 
 x=X(1);
 y=X(2);  
    
%% Solving 2D problem
 
%solving for theta3 (two possible solutions only)
a(1,3)=acos((-l3^2-l2^2+x^2+y^2)/(2*l2*l3)); %elbow down postive theta3
a(2,3)=-acos((-l3^2-l2^2+x^2+y^2)/(2*l2*l3)); %elbow up negative theta3 -> more stable

phi(1)= asin((sin(a(1,3))*l3)/(sqrt(x^2+y^2))); %intermediate variable needed to calculate theta2
phi(2)= asin((sin(a(2,3))*l3)/(sqrt(x^2+y^2)));

%calculating theta2 
a(1,2)=atan2(y,x)-phi(1);
a(2,2)=atan2(y,x)-phi(2);

a_deg=a*180/pi
end

