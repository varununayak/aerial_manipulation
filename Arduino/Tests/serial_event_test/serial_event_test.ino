const byte numChars = 32;
char receivedChars[numChars];

 float f1=0.0;
 float f2=0.0;
 float f3=0.0;



boolean newData = false;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {
    Serial.print(f1);
    Serial.print(',');
    Serial.print(f2);
    Serial.print(',');
    Serial.println(f3);
}

void serialEvent()
{
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
  f1=atof(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");
  f2=atof(strtokIndx);
  strtokIndx = strtok(NULL, "\0");
  f3=atof(strtokIndx);
  showNewData();
  }
}

void showNewData() {
  if(newData){
   //Serial.println(f1);
    //Serial.println(f2);
     //Serial.println(f3);
     newData=false;
  }
}
