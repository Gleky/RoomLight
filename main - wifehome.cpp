#include <Arduino.h>

const int main_pwmPin = 9,
          main_potPin = A7;


void update();
/////////////////--SETUP--//////////////////////////
void setup() {
  // Serial.begin(9600);

// Пины D9 и D10 - 31.4 кГц
  // TCCR1A = 0b00000001;  // 8bit
  // TCCR1B = 0b00000001;  // x1 phase correct
  // Пины D9 и D10 - 62.5 кГц
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00001001;  // x1 fast pwm

  analogWrite(main_pwmPin, 0);

  while (true)
  {
    update();
    delay(30);
  }
}

/////////////////--LOOP--////////////////////////////
void loop() {}

/////////////////--SUBFUNCTIONS--///////////////////////////
void update()
{
  static int oldPotVal = 0;
  int potValue = analogRead(main_potPin);

  if ( abs(oldPotVal - potValue) < 4 )
    return;
  else
    oldPotVal = potValue;
  

  static const int lowDead = 50;
  static const int highDead = 950;

  int pwmValue = map(potValue, lowDead, highDead, 0, 255);

  if ( pwmValue < 0 )
    pwmValue = 0;
  else if ( pwmValue > 255 )
    pwmValue = 255;

  analogWrite(main_pwmPin, pwmValue);

  // Serial.print("potValue: ");
  // Serial.print(potValue);
  // Serial.print(", pwmValue: ");
  // Serial.println(pwmValue);
}



//--------энергосберегайки----17ma->5ma(5v) (ардуина)---------------
// void interrupt_main(){
//     mainClicked = true;
// }

// void interrupt_table(){
//     tableClicked = true;
// }
// void goSleep(){
//     attachInterrupt(digitalPinToInterrupt(main_swPin), interrupt_main, LOW);
//     attachInterrupt(digitalPinToInterrupt(table_swPin), interrupt_table, LOW);

//     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//     sleep_mode();

//     detachInterrupt(digitalPinToInterrupt(main_swPin));
//     detachInterrupt(digitalPinToInterrupt(table_swPin));
//     delay(100);
// }
//--------------------------------------------