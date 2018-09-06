#include "AutoPID.h"

AutoPID::AutoPID(float *input, int *setpoint, int *output1, int *output2, uint8_t outputMin, uint8_t outputMax, double Kp, double Ki, double Kd) {
  _input = input;
  _setpoint = setpoint;
  _output1 = output1;
  _output2 = output2;
  _outputMin = outputMin;
  _outputMax = outputMax;
  setGains(Kp, Ki, Kd);
  _timeStep = 1;
}//AutoPID::AutoPID
void AutoPID::setGains(double Kp, double Ki, double Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}//AutoPID::setControllerParams
void AutoPID::setOutputRange(uint8_t outputMin, uint8_t outputMax) {
  _outputMin = outputMin;
  _outputMax = outputMax;
}//void AutoPID::setOutputRange
void AutoPID::setTimeStep(unsigned long timeStep) {
  _timeStep = timeStep;
}
void AutoPID::run() {
  unsigned long _dT = millis() - _lastStep;   //calculate time since last update
  if (_dT >= _timeStep) {                     //if long enough, do PID calculations
    _lastStep = millis();
    if (*_setpoint > *_input)
    {
      double _error;
      if ((*_setpoint - *_input) < 200)
        _error = *_setpoint - *_input;
      else if ((*_setpoint - *_input) > 200)
      {
        *_input += 360;
        _error = abs(*_input - *_setpoint) ;
        *_input -= 360;
      }
      _integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
      //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
      double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
      _previousError = _error;
      double PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
      //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax - _outputMin));
      if ((*_setpoint - *_input) < 200)
      {
        *_output1 = constrain(PID, _outputMin, _outputMax);
        *_output2 = 0;
      }
      else if ((*_setpoint - *_input) > 200)
      {
        *_output1 = 0;
        *_output2 = constrain(PID, _outputMin, _outputMax);
      }
    }
    else if (*_setpoint < *_input)
    {
      double _error;
      if ((*_input - *_setpoint) < 200)
        _error = *_input - *_setpoint ;
      else if ((*_input - *_setpoint) > 200)
      {
        *_setpoint += 360;
        _error = abs(*_input - *_setpoint) ;
        *_setpoint -= 360;
      }
      _integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
      //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
      double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
      _previousError = _error;
      double PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
      //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax - _outputMin));
      if ((*_input - *_setpoint) < 200)
      {
        *_output2 = constrain(PID, _outputMin, _outputMax);
        *_output1 = 0;
      }
      else if ((*_input - *_setpoint) > 200)
      {
        *_output2 = 0;
        *_output1 = constrain(PID, _outputMin, _outputMax);
      }
    }
  }

}//void AutoPID::run
void AutoPID::reset() {
  _lastStep = millis();
  _integral = 0;
  _previousError = 0;
}


