/*
 * This arduino code reads three float values - theta1, theta2 and theta3 in degrees from serial port and writes one float value Force in Newtons to serial port
 * The force is sensed using capactivesensor library and filtered using ExpSmoothing/GaussianFilter
 * It also controls the servos using Servo library
 * 
 * Sample input from serial port for theta values-
 * <25.2,26.2,27.2>
 */


#include <CapacitiveSensor.h> //capsense library
#include <Servo.h>

// pin allocation
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 6;
const int SERVO3_PIN = 9;
const int CAP_SEND_PIN = 4;
const int CAP_RECEIVE_PIN =2 ;


///// force sensor setup
CapacitiveSensor   cs = CapacitiveSensor(CAP_SEND_PIN,CAP_RECEIVE_PIN);        // 10 megohm resistor between pins SEND & REC, pin REC  is sensor pin, add wire, foil


/////// INITIALIZE theta values ---calibrate manually and enter values here
float theta1_home=0;
float theta2_home=90;
float theta3_home=-180;
float theta1= theta1_home;
float theta2= theta2_home;
float theta3= theta3_home;


///SERVO SETUP
Servo servo1,servo2,servo3;


////SERIAL DATA READER STUFF
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

//
float F_filt = 0.0; //initialize filtered force output

//--------------------END OF PREAMBLE------------------------//


void setup() {
  //set pins here
 Serial.begin(115200);

 
pinMode(SERVO1_PIN,OUTPUT);
pinMode(SERVO2_PIN,OUTPUT);
pinMode(SERVO3_PIN,OUTPUT);
pinMode(CAP_SEND_PIN,OUTPUT);
pinMode(CAP_RECEIVE_PIN,OUTPUT);

//attach servos to pins
servo1.attach(SERVO1_PIN);
servo2.attach(SERVO2_PIN);
servo3.attach(SERVO3_PIN);


cs.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
}




void loop() {
float F=0;
control_servos(); //sends PWM command to servos based on theta vals
F = get_force(); //updates force value read from capacitive sensor (filtered)
write_serial_data(F); //writes force value to serial port

}




float get_force()
{
  // get raw capacitor reading, sense and return
// float F_raw =  cs.capacitiveSensor(30);
 //F_filt = filter(F_raw,F_filt); //filter raw reading

 F_filt = 0.5;

 return F_filt;  //return filtered value
}







void control_servos()
{
  //convert theta vals to PWM signal and servo.write them
 int servomicro1, servomicro2, servomicro3;

servomicro1 = mapang(theta1, -90, 90, 2255, 780);  // constantly update the mapped values (calibration)
servomicro2 = mapang(theta2, 0, 180, 2255, 765); 
servomicro3 = mapang(theta3, -180, 0, 900, 1920);

servomicro1= constrain(servomicro1, 800, 2200); //safety and prevent overkill 
servomicro2= constrain(servomicro2, 780, 2200);
servomicro3= constrain(servomicro3, 900, 1900);


 servo1.writeMicroseconds(servomicro1);
 servo2.writeMicroseconds(servomicro2);
 servo3.writeMicroseconds(servomicro3); 
 
 //Serial.print(theta1);
 //Serial.print(',');
 //Serial.print(theta2);
 //Serial.print(',');
 //Serial.println(theta3);
}



//-----------WRITE TO SERIAL---------------//

void write_serial_data(float val) //this writes the force value to serial port
{
Serial.print('<'); //startmarker
Serial.print(val);
Serial.println('>'); //endmarker
}


//-------------------SERIAL READER-----------------//


void serialEvent()
{
//parse float values from serial port to be read into theta1, theta2 and theta3
noInterrupts();
 recvWithStartEndMarkers();
 parseData();
interrupts();
}



void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
         
}


void parseData()
{
  if(newData)
  {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(receivedChars, ",");
  theta1=atof(strtokIndx);  
  strtokIndx = strtok(NULL, ",");
  theta2=atof(strtokIndx);
  strtokIndx = strtok(NULL, "\0");
  theta3=atof(strtokIndx);
  newData=false;
  }
}


//-----------------END of SERIAL READER-----------------------------//




//FILTER FOR FORCE SENSOR
float filter(float F_raw, float F_filt)
{
  //exponential smoothing filter
  float k=0.5;
 float F_aftfilt= k*F_raw+(1-k)*F_filt;
  return F_aftfilt;
}


//ANGLE MAPPED TO MICROSECONDS , did not use map() function to preseve float accuracy
int mapang(float theta, float xi, float xf, float yi, float yf)
{
  int y = (yf-yi)/(xf-xi)*(theta-xi) +yi;
  return int(y);
}








