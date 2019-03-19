% -This code loads all design parameters into 'parameters.mat' - link lengths
%  and end effector pose in frame 3. DH parameters are derived from them.
% -MAKE SURE YOU SAVE AND RUN THE CODE WHENEVER YOU CHANGE ANYTHING!
% -ONLY SI UNITS

clear all

%% Link Design Parameters
l1=0.100; %in m
l2=0.300;
l3=0.200;

dl1=0.015; %offset 1 (due to servo2)
dl2=0.010; %offset 2 (due to servo3)

%% EE position and orientation specification in frame 3
r11=1;r12=0;r13=0; %elements of Orientation part of Transformation matrix
r21=0;r22=1;r23=0;
r31=0;r32=0;r33=1;
ex=0;ey=0;ez=0; %end effector position (final point) expressed in frame 3
ee_pos=[ex;ey;ez;1];

%% DH Parameters
a1=dl1;d1=l1;alpha1=pi/2; %theta1,theta2,theta3 are variables so it is excluded
a2=l2;d2=0;alpha2=0;
a3=l3;d3=0;alpha3=0;


%% 
save('parameters')