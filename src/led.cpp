#include "led.hpp"
#include <Arduino.h>

Led::Led( int pwmPin )
    : _pwmPin( pwmPin )
{
    pinMode( _pwmPin, OUTPUT );
    digitalWrite( _pwmPin, 0 );
}
