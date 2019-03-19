clear,clc
close all

m=2.6;
l=0.3; %moment arm
L=0.8; %quad characteristic length (side of square)
g=9.81;  
I=0.10; %quad moment of inertia about pitch axis


%% Desired values !!

thetades=0.32;
xdes=0;

%% PD control gains !!

% Theta Control
kp1=3;  %2.5  
kd1=1.2;  %1.2

% X control
kp2=50; % 50 
kd2=10;    % 10     keep this low because of ignored xddd term                                              

%% Formulation of Force, T1 and T2 from control law 

syms x(t) theta(t)

F=m*g*tan(theta) - m*diff(x,t,t);

sigma1=m*(F/m-kd1*diff(x,t)+kp1*(xdes-x))/(2*sin(theta));

sigma2=I*(kp2*(thetades-theta)  -kd2*diff(theta)...
    + (l*F)/I)/(L);
T1 = sigma1-sigma2;
T2 = sigma1+sigma2;



%% The differential equation to be solved (closed-loop dynamics)

diffeqns=[diff(x,t,t)  ==  -F/m+(sin(theta))*(T1+T2)/m;...
    diff(theta,t,t) ==  -F*l/I+(T2-T1)*L/(2*I)]; %diff(theta,t,t) expression

% formulation
[V] = odeToVectorField(diffeqns);
M = matlabFunction(V,'vars', {'t','Y'});

%% Initial conditions!

x0=         -0.05;
xdot0=      0.10;
theta0=     0.1;
thetadot0=  0.2;

%% Solving...

%using ode45 solver
sol = ode45(M,[0 4],[theta0 thetadot0 x0 xdot0]); %ode23s is good

% Storing the answers
time=double(sol.x);
xsol=double(sol.y(3,:));
xdsol=double(sol.y(4,:));
thetasol=double(sol.y(1,:));
thetadsol=double(sol.y(2,:));

xddsol=diff(xdsol);
xddsol(length(xddsol)+1)=xddsol(length(xddsol)); %to make the lengths same for plotting

Fsol=m*g*tan(thetasol)-m*xddsol; %from expression for force 

% Intializing the thrust values
T1sol=zeros(1,length(time));
T2sol=zeros(1,length(time));

% Calculating T1 and T2
sigma1sol=m*(Fsol/m-kd1*xdsol+kp1*(xdes-xsol))/(2*sin(thetasol));
sigma2sol=I*(kp2*(thetades-thetasol)  -kd2*thetadsol...
    + (l*Fsol)/I)/(L);

T1sol = sigma1sol-sigma2sol;
T2sol = sigma1sol+sigma2sol;


%% Plotting useful values w.r.t. time

subplot(3,1,1)
plot(time,xsol,time,thetasol);
legend('x','theta');
xlabel('Time (s)');
ylabel('SI Units');

subplot(3,1,2)
plot(time,Fsol);
legend('F');
xlabel('Time (s)');
ylabel('Newton');
%%{
subplot(3,1,3)

plot(time,T1sol,time,T2sol,time,(T1sol+T2sol).*cos(thetasol));
legend('T1','T2','Total Thrust');
xlabel('Time (s)');
ylabel('Thrust (Newton)');
axis([0 4 0 50])
%%}



