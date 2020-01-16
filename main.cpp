#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>

#include <GyverEncoder.h>

void allLightFullUp();
void mainLightUp();
void mainLightDown();
void tableLightUp();
void tableLightDown();
void mainClick();
void tableClick();
void update();
int brightness(int linearBrightness);
void mainWarning();
void tableWarning();

const int fullLightUpTime = 2 *1000;

const int step = 16;

const int main_pwmPin = 9,
          table_pwmPin = 10,

          main_clkPin = 4,
          main_dtPin = 5,
          main_swPin = 2,

          table_clkPin = 6,
          table_dtPin = 7,
          table_swPin = 3;

volatile bool mainClicked = false,
              tableClicked = false;

volatile int mainTargetPWM = 0,
             tableTargetPWM = 0,
             mainCurrentPWM = 0,
             tableCurrentPWM = 0;

Encoder mainEnc(main_clkPin,main_dtPin,main_swPin, TYPE1);
Encoder tableEnc(table_clkPin,table_dtPin,table_swPin, TYPE1);


/////////////////--SETUP--//////////////////////////
void setup() {
  analogWrite(main_pwmPin, 0);
  analogWrite(table_pwmPin, 0);
  allLightFullUp();

  // Serial.begin(9600);
}

/////////////////--LOOP--////////////////////////////
void loop() {
  mainEnc.tick();
  tableEnc.tick();
  update();

  if (mainEnc.isRight())
    mainLightUp();
  else if (mainEnc.isLeft())
    mainLightDown();

  if (tableEnc.isLeft())
    tableLightUp();
  else if (tableEnc.isRight())
    tableLightDown();

  if (mainEnc.isClick())
    mainClicked = true;
  if (tableEnc.isClick())
    tableClicked = true;

  if (mainClicked)
    mainClick();
  if (tableClicked)
    tableClick();

  delay(1);
}

/////////////////--SUBFUNCTIONS--///////////////////////////
void update(){
  static unsigned long lastTime = 0;
  static const unsigned long period = fullLightUpTime/255;

  if ((millis() - lastTime) < period)
    return;

  lastTime = millis();

  bool mainOk = false;
  bool tableOk = false;

  if (mainTargetPWM > mainCurrentPWM)
    ++mainCurrentPWM;
  else if (mainTargetPWM < mainCurrentPWM)
    --mainCurrentPWM;
  else
    mainOk = true;

  if (tableTargetPWM > tableCurrentPWM)
    ++tableCurrentPWM;
  else if (tableTargetPWM < tableCurrentPWM)
    --tableCurrentPWM;
  else
    tableOk = true;

  if (tableOk && mainOk)
    return;

  analogWrite(main_pwmPin, brightness(mainCurrentPWM));
  analogWrite(table_pwmPin, brightness(tableCurrentPWM));
}

int brightness(int linearBrightness){

  // Serial.print("in : ");
  // Serial.println(linearBrightness);

  long ret = linearBrightness;
  ret = ret * ret;

  // Serial.print("mid : ");
  // Serial.println(ret);

  ret = ret / 256;

  // Serial.print("out : ");
  // Serial.println(ret);
  // Serial.println();

  if (ret < 0)
    ret = 0;
  if (ret > 253)
    ret = 255;

  return ret;
}

void mainLightUp(){
  mainTargetPWM += step;
  if (mainTargetPWM > 255)
    mainTargetPWM = 255;
}
void mainLightDown(){
  mainTargetPWM -= step;
  if (mainTargetPWM < 0)
    mainTargetPWM = 0;
}

void tableLightUp(){
  tableTargetPWM += step;
  if (tableTargetPWM > 255)
    tableTargetPWM = 255;
}
void tableLightDown(){
  tableTargetPWM -= step;
  if (tableTargetPWM < 0)
    tableTargetPWM = 0;
}

void mainClick(){
  if (mainTargetPWM > 0)
    mainTargetPWM = 0;
  else
    mainTargetPWM = 255;

  mainClicked = false;
}
void tableClick(){
  if (tableTargetPWM > 0)
    tableTargetPWM = 0;
  else
    tableTargetPWM = 255;

  tableClicked = false;
}

void allLightFullUp(){
  tableTargetPWM = 255;
  mainTargetPWM = 255;
}

void tableWarning(){
  digitalWrite(table_pwmPin, 0);
  delay(200);
  digitalWrite(table_pwmPin, 1);
  delay(200);
  digitalWrite(table_pwmPin, 0);
  delay(200);
  digitalWrite(table_pwmPin, 1);
}

void mainWarning(){
  digitalWrite(main_pwmPin, 0);
  delay(200);
  digitalWrite(main_pwmPin, 1);
  delay(200);
  digitalWrite(main_pwmPin, 0);
  delay(200);
  digitalWrite(main_pwmPin, 1);
}

//--------энергосберегайки----17ma->5ma(5v) (ардуина)---------------
void interrupt_main(){
    mainClicked = true;
}

void interrupt_table(){
    tableClicked = true;
}
void goSleep(){
    attachInterrupt(digitalPinToInterrupt(main_swPin), interrupt_main, LOW);
    attachInterrupt(digitalPinToInterrupt(table_swPin), interrupt_table, LOW);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();

    detachInterrupt(digitalPinToInterrupt(main_swPin));
    detachInterrupt(digitalPinToInterrupt(table_swPin));
    delay(100);
}
//--------------------------------------------