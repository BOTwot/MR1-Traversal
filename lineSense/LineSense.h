#include <Arduino.h>

class LineSense
{
    private:
    uint8_t *_sen1,*_sen2;
    float *_currangle;
    int16_t *_setpoint;
    int32_t *_pwmfw, *_pwmbk, *_pwmlt, *_pwmrt,
    public:
    LineSense(uint8_t *sen1,uint8_t *sen2,float *currangle,int16_t *setpoint, int32_t *pwmfw, int32_t *pwmbk, int32_t *pwmlt, int32_t *pwmrt,)
    {
        _sen1=sen1;
        _sen2=sen2;
        _currangle=currangle;
        _setpoint=setpoint;
        _pwmfw=pwmfw;
        _pwmbk=pwmbk;
        _pwmlt=pwmlt;
        _pwmrt=pwmrt;

    }
    void run()
    {
        B00011000
        //left adj
        if(((*_sen2|B10000001)==B11111111)||((*_sen2|B10000001)==B10000001))
        {
            *_setpoint=*_currangle;
            if(*_sen1==B00110000||*_sen1==B01100000||*_sen1==B11000000||*_sen1==B10000000)
            {
                _pwmlt+=50;
            }
            else if(*_sen1==B00111000||*_sen1==B01110000||*_sen1==B11100000)
            {
                _pwmlt+=50;
            }
            else if(*_sen1==B00011100||*_sen1==B00001110||*_sen1==B00000111) 
            {
                _pwmrt+=50;
            }
            else if(*_sen1==B00000001||*_sen1==B00001100||*_sen1==B00000110||*_sen1==B00000011)
            {
                _pwmrt+=50;
            }
        
        }
        else 
        {
            
        }


}