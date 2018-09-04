//Libraries Included
#include <FlySkyIBus.h>
#include <Wire.h>
#include "AutoPID.h"
#include <I2C_Anything.h>
#include "GyroRead.h"
//Definitions of pins and Variables
#define NOSE_PIN1 3
#define NOSE_PIN2 5
#define LEFT_PIN1 6
#define LEFT_PIN2 9
#define RIGHT_PIN1 10
#define RIGHT_PIN2 11
//Global Varibles declared and somewhat defined
int stcorr1 = 0, stcorr2 = 0, sdlcorr1 = 0, sdlcorr2 = 0, sdrcorr1 = 0, sdrcorr2 = 0;   //Variables for AutoPID only
double stkp = 6, stki = 0.03, stkd = 150, sdkp = 2, sdki = 0.06, sdkd = 150;    //Kp,Ki,Kd for AutoPID Lib
uint8_t stmax = 100, stmin = 0, sdmax = 100, sdmin = 0;    //Variables for min and max adjust pwm
float curangle;
int prevangle = 0;
int32_t pwmfw, pwmbk, pwmlt, pwmrt, pwm, pwmthr;      //pwm variables for each direction
//Objects created for included libraries
GyroRead gyro;                //Creating object for GyroRead Lib
AutoPID straight(&curangle, &prevangle, &stcorr1, &stcorr2, stmin, stmax, stkp, stki, stkd);      //AutoPID Obj
AutoPID sidewayl(&curangle, &prevangle, &sdlcorr1, &sdlcorr2, sdmin, sdmax, sdkp, sdki, sdkd);    //AutoPID Obj
AutoPID sidewayr(&curangle, &prevangle, &sdrcorr1, &sdrcorr2, sdmin, sdmax, sdkp, sdki, sdkd);    //AutoPID Obj
//AutoPID adjust
void setup() {
  Serial.begin(115200);
  pinMode(NOSE_PIN1, OUTPUT);
  pinMode(NOSE_PIN2, OUTPUT);
  pinMode(LEFT_PIN1, OUTPUT);
  pinMode(LEFT_PIN2, OUTPUT);
  pinMode(RIGHT_PIN1, OUTPUT);
  pinMode(RIGHT_PIN2, OUTPUT);
  Wire.begin();
  IBus.begin(Serial1);  //Library Function
  gyro.begin(8);        //Library Function
  straight.setTimeStep(50);     //AutoPID Lib Functon
  sidewayl.setTimeStep(50);      //AutoPID Lib Function
  sidewayr.setTimeStep(50);      //AutoPID Lib Function
}
//Functions declaration and definition


void mapping()      //Call the function for mapping the pwm recieved from the remote
{
  pwm = map(pwm, 0, 500, 0, 255);
  pwmfw = map(pwmfw, 0, 500, 0, 255);
  pwmbk = map(pwmbk, 0, 500, 0, 255);
  pwmlt = map(pwmlt, 0, 500, 0, 255);
  pwmrt = map(pwmrt, 0, 500, 0, 255);

}
void mapforthr()
{
  pwmthr = map(pwmthr, 1000, 1600, 0, 255);
  Serial.println(pwmthr);
}
void updateangle()
{
  curangle = gyro.getAngle();
  prevangle = curangle;
  if (abs(prevangle - curangle) > 150)
  {
    if (prevangle > 300)
      prevangle -= 360;
    else if (curangle > 300)
      curangle -= 360;
  }
}
void stoped()       //Call the function to stop the movement
{
  analogWrite(LEFT_PIN2, 0);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(RIGHT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(NOSE_PIN2, 0);
  analogWrite(NOSE_PIN1, 0);
  pwmfw = pwmbk = pwmlt = pwmrt = pwm = 0;    //just to be on safe side so it will not give prev pwm when forward called
  sdrcorr1 = sdrcorr2 = sdlcorr1 = sdlcorr2 = stcorr1 = stcorr2 = 0;    //If not written then it does not become 0 when stopped and bot "Gandtay"

  straight.reset();    //used to set millis to 0, improves Ki adjustment
  sidewayl.reset();
  sidewayr.reset();

}
void forward()      //Call the function for forward movement, adjust is included
{
  if ((stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2) > 0)
  {
    analogWrite(NOSE_PIN1, stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2);
    analogWrite(NOSE_PIN2, 0);
    //Serial.print("if ");
    //Serial.println(stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2);

  }
  else if ((stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2) < 0)
  {
    analogWrite(NOSE_PIN1, 0);
    analogWrite(NOSE_PIN2, stcorr2 + pwmlt + sdlcorr2 - stcorr1 - pwmrt - sdrcorr1 );
    //Serial.print("in else ");
    //Serial.println(stcorr2 + pwmlt + sdlcorr2 - stcorr1 - pwmrt - sdrcorr1);
    //Serial.print("out else ");
  }
  if ((pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk) > 0)
  {
    analogWrite(LEFT_PIN1, pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk);
    analogWrite(LEFT_PIN2, 0);
    //Serial.print("if ");
    //Serial.println(pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk);
  }
  else if ((pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk) < 0)
  {
    analogWrite(LEFT_PIN1, 0);
    analogWrite(LEFT_PIN2, 0.5 * pwmrt + pwmbk - pwmfw - 0.5 * pwmlt - sdlcorr1 );
    //Serial.print("in else left ");
    //Serial.println(0.5 * pwmrt + pwmbk - pwmfw - 0.5 * pwmlt - sdlcorr1);
  }
  if ((pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk) > 0)
  {
    analogWrite(RIGHT_PIN1, pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk);
    analogWrite(RIGHT_PIN2,  0);
    //Serial.print("if right ");
    //Serial.println(pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk);
  }
  else if ((pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk) < 0)
  {
    analogWrite(RIGHT_PIN1, 0);
    analogWrite(RIGHT_PIN2, 0.5 * pwmlt + pwmbk - 0.5 * pwmrt - sdrcorr2 - pwmfw);
    //Serial.print("in else rigth ");
    //Serial.println(0.5 * pwmlt + pwmbk - 0.5 * pwmrt - sdrcorr2 - pwmfw);
  }
}
void cwise()        //Call the function for clock wise movement
{
  analogWrite(NOSE_PIN1, pwm);
  analogWrite(NOSE_PIN2, 0);
  analogWrite(LEFT_PIN1, pwm);
  analogWrite(LEFT_PIN2, 0);
  analogWrite(RIGHT_PIN1, 0);
  analogWrite(RIGHT_PIN2, pwm);
}
void ccwise()        //Call the function for counter-clock wise movement
{
  analogWrite(NOSE_PIN1, 0);
  analogWrite(NOSE_PIN2, pwm);
  analogWrite(LEFT_PIN1, 0);
  analogWrite(LEFT_PIN2, pwm);
  analogWrite(RIGHT_PIN1, pwm);
  analogWrite(RIGHT_PIN2, 0);
}
void loop() {
  IBus.loop();
  if (IBus.readChannel(4) == 1000) //Use to cut off batttery with relay
  {
    Serial.println("OFF");
    stoped();
  }
  else if (IBus.readChannel(5) == 2000) // Throttle Mode
  {
    IBus.loop();    //Call always in loop to recieve signal from remote
    //Condition for stop below
    if ((IBus.readChannel(3) >= 1465 && IBus.readChannel(3) <=  1530) &&
        (IBus.readChannel(0) >= 1465 && IBus.readChannel(0) <=  1535) &&
        (IBus.readChannel(1) >= 1465 && IBus.readChannel(1) <=  1535))
    {
      stoped();
      updateangle();
      //Serial.println(sdlcorr1);
      //Serial.println(sdlcorr2);
      //Serial.println(pwmfw);
      //Serial.println(pwmbk);
    }
    pwmthr = IBus.readChannel(2);
    mapforthr();
    if (IBus.readChannel(3) > 1535)     //rotation clockwise
    {
      IBus.loop();
      pwm = abs(IBus.readChannel(3) - 1500 );
      pwm = pwm * pwmthr / 500;
      mapping();
      cwise();
      updateangle();
    }
    if (IBus.readChannel(3) < 1465)     //rotation anti-clockwise
    {
      IBus.loop();
      pwm = abs( 1500 - IBus.readChannel(3) );
      pwm = pwm * pwmthr / 500;
      mapping();
      ccwise();
      updateangle();
    }
    while (IBus.readChannel(1) > 1535)     //forward
    {
      pwmthr = IBus.readChannel(2);
      mapforthr();
      IBus.loop();
      if (IBus.readChannel(0) > 1535) //right
      {
        pwmrt = abs(IBus.readChannel(0) - 1500);
        pwmrt = pwmrt * pwmthr / 500;
        sidewayr.run();
      }
      if (IBus.readChannel(0) < 1465)  //left
      {
        pwmlt = abs(1500 - IBus.readChannel(0));
        pwmlt = pwmlt * pwmthr / 500;
        sidewayl.run();
      }
      straight.run();
      pwmfw = abs(IBus.readChannel(1) - 1500 );
      pwmfw = pwmfw * pwmthr / 500;
      mapping();

      // Serial.println(pwmfw);
      forward();
      curangle = gyro.getAngle();
    }
    while (IBus.readChannel(1) < 1465)     //back
    {
      pwmthr = IBus.readChannel(2);
      mapforthr();
      IBus.loop();
      if (IBus.readChannel(0) > 1535) //right
      {
        pwmrt = abs(IBus.readChannel(0) - 1500);
        pwmrt = pwmrt * pwmthr / 500;
        sidewayr.run();
      }
      if (IBus.readChannel(0) < 1465)  //left
      {
        pwmlt = abs(1500 - IBus.readChannel(0));
        pwmlt = pwmlt * pwmthr / 500;
        sidewayl.run();
      }
      straight.run();
      pwmbk = abs(1500 - IBus.readChannel(1) );
      pwmbk = pwmbk * pwmthr / 500;
      mapping();
      //Serial.println(pwmbk);
      forward();
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) > 1535)     //right
    {
      IBus.loop();
      sidewayr.run();
      pwmrt = abs(IBus.readChannel(0) - 1500);
      pwmrt = pwmrt * pwmthr / 500;
      mapping();
      // Serial.println(pwmrt);
      forward();
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) < 1465)     //left
    {
      IBus.loop();
      sidewayl.run();
      pwmlt = abs(1500 - IBus.readChannel(0));
      pwmlt = pwmlt * pwmthr / 500;
      mapping();
      //Serial.println(pwmlt);
      forward();
      curangle = gyro.getAngle();
    }
  }
  else if (IBus.readChannel(7) == 2000) //Line tracing mode
  {
    Serial.println("Line Tracing Mode");
    if ((IBus.readChannel(3) >= 1465 && IBus.readChannel(3) <=  1530) &&
        (IBus.readChannel(0) >= 1465 && IBus.readChannel(0) <=  1535) &&
        (IBus.readChannel(1) >= 1465 && IBus.readChannel(1) <=  1535))
    {
      stoped();
      updateangle();
    }
  }
  else  // Defalut basic code
  {
    // Serial.println(IBus.readChannel(5));
    Serial.println("Default");
    IBus.loop();    //Call always in loop to recieve signal from remote
    //Condition for stop below
    if ((IBus.readChannel(3) >= 1465 && IBus.readChannel(3) <=  1530) &&
        (IBus.readChannel(0) >= 1465 && IBus.readChannel(0) <=  1535) &&
        (IBus.readChannel(1) >= 1465 && IBus.readChannel(1) <=  1535))
    {
      stoped();
      updateangle();
      Serial.println(sdlcorr1);
      Serial.println(sdlcorr2);
      //Serial.println(pwmfw);
      //Serial.println(pwmbk);
    }
    if (IBus.readChannel(3) > 1535)     //rotation clockwise
    {
      IBus.loop();
      pwm = abs(IBus.readChannel(3) - 1500 );
      mapping();
      cwise();
      updateangle();
    }
    if (IBus.readChannel(3) < 1465)     //rotation anti-clockwise
    {
      IBus.loop();
      pwm = abs( 1500 - IBus.readChannel(3) );
      mapping();
      ccwise();
      updateangle();
    }
    while (IBus.readChannel(1) > 1535)     //forward
    {
      IBus.loop();
      if (IBus.readChannel(0) > 1535) //right
      {
        pwmrt = abs(IBus.readChannel(0) - 1500);
        sidewayr.run();
      }
      if (IBus.readChannel(0) < 1465)  //left
      {
        pwmlt = abs(1500 - IBus.readChannel(0));
        sidewayl.run();
      }
      straight.run();
      pwmfw = abs(IBus.readChannel(1) - 1500 );
      mapping();

      // Serial.println(pwmfw);
      forward();
      pwmlt = pwmrt = pwm = 0;
      curangle = gyro.getAngle();
    }
    while (IBus.readChannel(1) < 1465)     //back
    {
      IBus.loop();
      if (IBus.readChannel(0) > 1535) //right
      {
        pwmrt = abs(IBus.readChannel(0) - 1500);
        sidewayr.run();
      }
      if (IBus.readChannel(0) < 1465)  //left
      {
        pwmlt = abs(1500 - IBus.readChannel(0));
        sidewayl.run();
      }
      straight.run();
      pwmbk = abs(1500 - IBus.readChannel(1) );
      mapping();
      //Serial.println(pwmbk);
      forward();
      pwmlt = pwmrt = pwm = 0;
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) > 1535)     //right
    {
      IBus.loop();
      sidewayr.run();
      pwmrt = abs(IBus.readChannel(0) - 1500);
      mapping();
      // Serial.println(pwmrt);
      forward();
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) < 1465)     //left
    {
      IBus.loop();
      sidewayl.run();
      pwmlt = abs(1500 - IBus.readChannel(0));
      mapping();
      //Serial.println(pwmlt);
      forward();
      curangle = gyro.getAngle();
    }
  }
}
