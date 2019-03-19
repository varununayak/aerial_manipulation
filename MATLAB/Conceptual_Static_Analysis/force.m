function F = force(y)
% -This function used along with forcecall.m calculates the thrusts
% required during contact. This is a static analysis done for conceptual
% purposes
% -Assumes no shift in CoG as m<<M

M=2.4; %1.2 quad mass
m=0.35; %0.6 manipul mass
g=9.81;
L=0.65; %0.6 %frame char length F550

Fv=3;   %-6 to +6 %vertical force on wall (check sign convention in figure)
Fn=8;   %horizontal push force 

l1=0.3; %moment arm for push force
l2=0.5; %moment arm for vertical force

%developing the non-linear equations
F(1) = (y(1)+y(2))*cos(y(3))+Fv-(M+m)*g; 
F(2) = (y(1)+y(2))*sin(y(3))-Fn;
F(3) = (y(1)-y(2))*L/2 -Fn*l1*cos(y(3))-Fv*(l2+l1*sin(y(3)));
end

