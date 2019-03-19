These are sample theta input values to the serial port of the arduino to check:

1) That the arduino successfully reads and parses the serial information

2) That the arduino successfully utilizes the theta values to control the manipulator



Format:

<theta1,theta2,theta3>  [float values]




Samples:



<25.22,26.21,-27.20>


<0,21,-50>

<4,90,-100>


<-10, 95, -150>


<22, 33, -56>


<25.2,26.2,27.2>


The force will be written to the serial port in the format <F(N)> ... Eg: <1.5> 