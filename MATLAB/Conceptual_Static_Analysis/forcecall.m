func=@force;
y0=[0,0,0];

y=fsolve(func,y0);
y(1) %T1 (thrust at the back)
y(2) %T2 (thrust at the front)
y(3)*180/pi %pitch angle in degrees