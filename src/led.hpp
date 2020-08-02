#pragma once

class Led
{
public:
    Led( int pwmPin );
    void update();

    void changeBrightness( int newBrightness );

private:
    int _currentValue = 0;
    int _targetValue = 0;

    const int _pwmPin;
};
