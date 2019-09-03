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
#define RIGHT_PIN1 12
#define RIGHT_PIN2 11

//Global Varibles declared and somewhat defined
int yawmaxpwm = 200, travmaxpwm = 240;    //PWM values for traversal and yaw
int switchOff = 1000, switchOn = 2000;    //Variables for channel switches
int minmarg = 1465, maxmarg = 1535;       //Variables for start margin to avoid extreme sensitivity for joysticks
int stcorr1 = 0, stcorr2 = 0, sdlcorr1 = 0, sdlcorr2 = 0, sdrcorr1 = 0, sdrcorr2 = 0;   //Variables for AutoPID only
double stkp = 3, stki = 0.0003, stkd = 275;   //Kp,Ki,Kd for AutoPID Lib
uint8_t stmax = 100, stmin = 0;    //Variables for min and max adjust pwm
float curangle, checkangle;
int prevangle = 0;    //pwm variables for each direction
int32_t pwmfw, pwmbk, pwmlt, pwmrt, pwm, pwmthr;      //32 bit int used to avoid the overflow in throttle mode

//Objects created for included libraries
GyroRead gyro;                //Creating object for GyroRead Lib
AutoPID straight(&curangle, &prevangle, &stcorr1, &stcorr2, stmin, stmax, stkp, stki, stkd);      //AutoPID Obj

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
  straight.setTimeStep(5);     //AutoPID Lib Functon
}
//Functions declaration and definition


void mapping()      //Call the function for mapping the pwm recieved from the remote
{
  pwm = map(pwm, 0, 500, 0, yawmaxpwm);   //to decrease the speed of yaw
  pwmfw = map(pwmfw, 0, 500, 0, travmaxpwm);   // can increase to max
  pwmbk = map(pwmbk, 0, 500, 0, travmaxpwm);
  pwmlt = map(pwmlt, 0, 500, 0, travmaxpwm);
  pwmrt = map(pwmrt, 0, 500, 0, travmaxpwm);

}
void mapforthr()    //mapping for throttle, only till 1600 to avoid unnecessary movement of stick
{
  pwmthr = map(pwmthr, 1000, 1600, 0, 255);
  Serial.println(pwmthr);
}
void updateangle()    // remember to add fail safe code in here
{
  checkangle = (int)gyro.getAngle();
  if (checkangle == 555)
  {
    stoped();
    //Serial.println("Device disconnected");
  }
  else
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
    //Serial.println(curangle);
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

}
void forward()      //Call the function for forward movement, adjust is included
{
  if (pwmfw > 200 || pwmbk > 200 || pwmlt > 200 || pwmrt > 200)
  {
    stmax *= 1.25;
  }
  else
  {
    stmax = 100;
  }
  straight.run();
  if ((stcorr1 + pwmrt - stcorr2 - pwmlt) > 0)
  {
    analogWrite(NOSE_PIN1, stcorr1 + pwmrt - stcorr2 - pwmlt);
    analogWrite(NOSE_PIN2, 0);
    //Serial.print("if ");
    //Serial.println(stcorr1 + pwmrt + sdrcorr1 - stcorr2 - pwmlt - sdlcorr2);

  }
  else if ((stcorr1 + pwmrt - stcorr2 - pwmlt) < 0)
  {
    analogWrite(NOSE_PIN1, 0);
    analogWrite(NOSE_PIN2, stcorr2 + pwmlt - stcorr1 - pwmrt );
    //Serial.print("in else ");
    //Serial.println(stcorr2 + pwmlt + sdlcorr2 - stcorr1 - pwmrt - sdrcorr1);
    //Serial.print("out else ");
  }
  if ((pwmfw + stcorr1 + 0.5 * pwmlt  - 0.5 * pwmrt - pwmbk - stcorr2) > 0)
  {
    analogWrite(LEFT_PIN1, pwmfw + stcorr1 + 0.5 * pwmlt  - 0.5 * pwmrt - pwmbk - stcorr2);
    analogWrite(LEFT_PIN2, 0);
    //Serial.print("if ");
    //Serial.println(pwmfw + 0.5 * pwmlt + sdlcorr1 - 0.5 * pwmrt - pwmbk);
  }
  else if ((pwmfw + stcorr1 + 0.5 * pwmlt  - 0.5 * pwmrt - pwmbk - stcorr2) < 0)
  {
    analogWrite(LEFT_PIN1, 0);
    analogWrite(LEFT_PIN2,  0.5 * pwmrt + pwmbk + stcorr2 - pwmfw - stcorr1 - 0.5 * pwmlt );
    //Serial.print("in else left ");
    //Serial.println(0.5 * pwmrt + pwmbk - pwmfw - 0.5 * pwmlt - sdlcorr1);
  }
  if ((pwmfw + stcorr2 + 0.5 * pwmrt - stcorr1 - 0.5 * pwmlt  - pwmbk) > 0)
  {
    analogWrite(RIGHT_PIN1, pwmfw + stcorr2 + 0.5 * pwmrt - stcorr1 - 0.5 * pwmlt  - pwmbk);
    analogWrite(RIGHT_PIN2,  0);
    //Serial.print("if right ");
    //Serial.println(pwmfw - 0.5 * pwmlt + 0.5 * pwmrt + sdrcorr2 - pwmbk);
  }
  else if ((pwmfw + stcorr2 + 0.5 * pwmrt - stcorr1 - 0.5 * pwmlt  - pwmbk) < 0)
  {
    analogWrite(RIGHT_PIN1, 0);
    analogWrite(RIGHT_PIN2, stcorr1 + 0.5 * pwmlt  + pwmbk - pwmfw - stcorr2 - 0.5 * pwmrt );
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
  if (IBus.readChannel(4) == switchOff) //Use to cut off batttery with relay
  {
    Serial.println("OFF");
    stoped();
  }
  else if (IBus.readChannel(5) == switchOn) // Throttle Mode
  {
    IBus.loop();    //Call always in loop to recieve signal from remote
    //Condition for stop below
    checkangle = (int)gyro.getAngle();
    if (checkangle == 555)
    {
      stoped();
      //Serial.println("in stop , device disconnected");
    }
    if ((IBus.readChannel(3) >= minmarg && IBus.readChannel(3) <=  maxmarg) &&
        (IBus.readChannel(0) >= minmarg && IBus.readChannel(0) <=  maxmarg) &&
        (IBus.readChannel(1) >= minmarg && IBus.readChannel(1) <=  maxmarg))
    {
      stoped();
      // updateangle();
      // Serial.println(sdlcorr1);
      // Serial.println(sdlcorr2);
      // Serial.println(pwmfw);
      // Serial.println(pwmbk);
    }
    pwmthr = IBus.readChannel(2);
    mapforthr();
    if (IBus.readChannel(3) > maxmarg)     //rotation clockwise
    {
      IBus.loop();
      pwm = abs(IBus.readChannel(3) - 1500 );
      pwm = pwm * pwmthr / 500;
      mapping();
      cwise();
      updateangle();
    }
    if (IBus.readChannel(3) < minmarg)     //rotation anti-clockwise
    {
      IBus.loop();
      pwm = abs( 1500 - IBus.readChannel(3) );
      pwm = pwm * pwmthr / 500;
      mapping();
      ccwise();
      updateangle();
    }
    while (IBus.readChannel(1) > maxmarg)     //forward
    {
      checkangle = (int)gyro.getAngle();
      if (checkangle == 555)
      {
        stoped();
      }
      else
      {
        pwmthr = IBus.readChannel(2);
        mapforthr();
        IBus.loop();
        if (IBus.readChannel(0) > maxmarg) //right
        {
          pwmrt = abs(IBus.readChannel(0) - 1500);
          pwmrt = pwmrt * pwmthr / 500;
          // sidewayr.run();
        }
        if (IBus.readChannel(0) < minmarg)  //left
        {
          pwmlt = abs(1500 - IBus.readChannel(0));
          pwmlt = pwmlt * pwmthr / 500;
          //sidewayl.run();
        }
        straight.run();
        pwmfw = abs(IBus.readChannel(1) - 1500 );
        pwmfw = pwmfw * pwmthr / 500;
        mapping();

        // Serial.println(pwmfw);
        forward();
        curangle = gyro.getAngle();
      }
    }
    while (IBus.readChannel(1) < minmarg)     //back
    {
      checkangle = (int)gyro.getAngle();
      if (checkangle == 555)
      {
        stoped();
      }
      else
      {
        pwmthr = IBus.readChannel(2);
        mapforthr();
        IBus.loop();
        if (IBus.readChannel(0) > maxmarg) //right
        {
          pwmrt = abs(IBus.readChannel(0) - 1500);
          pwmrt = pwmrt * pwmthr / 500;
          // sidewayr.run();
        }
        if (IBus.readChannel(0) < minmarg)  //left
        {
          pwmlt = abs(1500 - IBus.readChannel(0));
          pwmlt = pwmlt * pwmthr / 500;
          // sidewayl.run();
        }
        straight.run();
        pwmbk = abs(1500 - IBus.readChannel(1) );
        pwmbk = pwmbk * pwmthr / 500;
        mapping();
        //Serial.println(pwmbk);
        forward();
        curangle = gyro.getAngle();
      }
    }
    if (IBus.readChannel(0) > maxmarg)     //right
    {
      IBus.loop();
      // sidewayr.run();
      pwmrt = abs(IBus.readChannel(0) - 1500);
      pwmrt = pwmrt * pwmthr / 500;
      mapping();
      // Serial.println(pwmrt);
      forward();
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) < minmarg)     //left
    {
      IBus.loop();
      // sidewayl.run();
      pwmlt = abs(1500 - IBus.readChannel(0));
      pwmlt = pwmlt * pwmthr / 500;
      mapping();
      //Serial.println(pwmlt);
      forward();
      curangle = gyro.getAngle();
    }
  }
  else  // Defalut basic code
  {
    // Serial.println(IBus.readChannel(5));
    Serial.println("Default");
    IBus.loop();    //Call always in loop to recieve signal from remote
    //Condition for stop below
    checkangle = (int)gyro.getAngle();
    if (checkangle == 555)
    {
      stoped();
      //Serial.println("in stop, device disconnected");
    }
    if ((IBus.readChannel(3) >= minmarg && IBus.readChannel(3) <=  maxmarg) &&
        (IBus.readChannel(0) >= minmarg && IBus.readChannel(0) <=  maxmarg) &&
        (IBus.readChannel(1) >= minmarg && IBus.readChannel(1) <=  maxmarg))
    {
      stoped();
      // updateangle();
      Serial.println(sdlcorr1);
      Serial.println(sdlcorr2);
      //Serial.println(pwmfw);
      //Serial.println(pwmbk);
    }
    if (IBus.readChannel(3) > maxmarg)     //rotation clockwise
    {
      IBus.loop();
      pwm = abs(IBus.readChannel(3) - 1500 );
      mapping();
      cwise();
      updateangle();
    }
    if (IBus.readChannel(3) < minmarg)     //rotation anti-clockwise
    {
      IBus.loop();
      pwm = abs( 1500 - IBus.readChannel(3) );
      mapping();
      ccwise();
      updateangle();
    }
    while (IBus.readChannel(1) > maxmarg)     //forward
    {
      checkangle = (int)gyro.getAngle();
      if (checkangle == 555)
      {
        stoped();
      }
      else
      {
        IBus.loop();
        if (IBus.readChannel(0) > maxmarg) //right
        {
          pwmrt = abs(IBus.readChannel(0) - 1500);
          // sidewayr.run();
        }
        if (IBus.readChannel(0) < minmarg)  //left
        {
          pwmlt = abs(1500 - IBus.readChannel(0));
          //sidewayl.run();
        }
        straight.run();
        pwmfw = abs(IBus.readChannel(1) - 1500 );
        mapping();

        // Serial.println(pwmfw);
        forward();
        pwmlt = pwmrt = pwm = 0;
        curangle = gyro.getAngle();
      }
    }
    while (IBus.readChannel(1) < minmarg)     //back
    {
      checkangle = (int)gyro.getAngle();
      if (checkangle == 555)
      {
        stoped();
      }
      else
      {
        checkangle = gyro.getAngle();
        IBus.loop();
        if (IBus.readChannel(0) > maxmarg) //right
        {
          pwmrt = abs(IBus.readChannel(0) - 1500);
          //sidewayr.run();
        }
        if (IBus.readChannel(0) < minmarg)  //left
        {
          pwmlt = abs(1500 - IBus.readChannel(0));
          //sidewayl.run();
        }
        straight.run();
        pwmbk = abs(1500 - IBus.readChannel(1) );
        mapping();
        //Serial.println(pwmbk);
        forward();
        pwmlt = pwmrt = pwm = 0;
        curangle = gyro.getAngle();
      }
    }
    if (IBus.readChannel(0) > maxmarg)     //right
    {
      IBus.loop();
      //sidewayr.run();
      pwmrt = abs(IBus.readChannel(0) - 1500);
      mapping();
      // Serial.println(pwmrt);
      forward();
      curangle = gyro.getAngle();
    }
    if (IBus.readChannel(0) < minmarg)     //left
    {
      IBus.loop();
      // sidewayl.run();
      pwmlt = abs(1500 - IBus.readChannel(0));
      mapping();
      //Serial.println(pwmlt);
      forward();
      curangle = gyro.getAngle();
    }
  }
}
