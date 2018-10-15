#include <Wire.h>
#include <I2C_Anything.h>
String inString = "";
float angle = 0;
uint16_t time1=0,time2=0;
//int st;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Serial.begin(9600);
}
void requestEvent() {
  I2C_writeAnything(angle);
}
void loop() {
  if(Serial.available() > 0)
 {
  time1=0;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar != '\n') {
      inString += (char)inChar;
    }
    else {
      angle = inString.toFloat();
      inString = "";
    }
    Wire.onRequest(requestEvent);
  }
  time1=millis();
  }
  else 
  {
    time2=millis();
    if(abs(time2-time1)>500)
    {
      angle=555;
    Wire.onRequest(requestEvent);
    }
  }
   
}

