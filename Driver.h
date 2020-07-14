/*--------- INCLUDE --------- */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

/*---------  DEFINE --------- */
#define SH_CP                 2
#define ST_CP                 3
#define DS                    4
#define PWMA                  5      /* IN1 */
#define PWMB                  6      /* IN3 */ 
#define A_C1                  7
#define A_C2                  8
#define B_C1                  9
#define BUT1                  10
#define BUT2                  11
#define BUT3                  12
#define B_C2                  13
#define SS1                   A7
#define SS2                   A6
#define SS3                   17
#define SS4                   16
#define SS5                   15
#define SS6                   14
#define SDA                   19
#define SCL                   18
#define RoundOneWheelTurn90Degree  880
#define MotorAError                205 
#define MotorBError                0
/*--------- GLOBAL PARA --------- */
LiquidCrystal_I2C lcd(0x27,16,2);
uint8_t shiftData = 255;
int averageADC[6];

/*--------- Functions --------- */
int isBut(int index);
void writeLCD(int x, int y, String data);
int learnColor();
void setShiftIC();
void setLED(uint8_t ledPos);
uint8_t getSensor(uint8_t amount);
void setMotor(int mA, int mB);
void turn90_Left();
void turn90_Right();
void turn90_2Motors(int direct);

/*--------- Define Functions --------- */
int isBut(int index)
{
  switch (index)
  {
    case 1:
       if (!digitalRead(BUT1))
         delay(100);
       return !digitalRead(BUT1);
    case 2:
       if (!digitalRead(BUT2))
        delay(100);
      return !digitalRead(BUT2);
    case 3:
       if (!digitalRead(BUT3))
        delay(100);
      return !digitalRead(BUT3);
       return 0;
  }
}

void writeLCD(int x, int y, String data)
{
  lcd.setCursor(x,y);
  lcd.print(data);
}

int learnColor()
{
  int maxADC[6], minADC[6];
  uint8_t tempAverageADC;
  // Clear and print to lcd
  lcd.clear();
  writeLCD(1,0, "Learning Color");

  // Create initial for array
  for (int i=0; i<6; i++)
  {
    maxADC[i]=-1;
    minADC[i]=1025;
  }

  // Get ADC values
  while(!isBut(3))
  {
    if (maxADC[0] < analogRead(SS1)) maxADC[0] = analogRead(SS1);
    if (maxADC[1] < analogRead(SS2)) maxADC[1] = analogRead(SS2);
    if (maxADC[2] < analogRead(SS3)) maxADC[2] = analogRead(SS3);
    if (maxADC[3] < analogRead(SS4)) maxADC[3] = analogRead(SS4);
    if (maxADC[4] < analogRead(SS5)) maxADC[4] = analogRead(SS5);
    if (maxADC[5] < analogRead(SS6)) maxADC[5] = analogRead(SS6);

    if (minADC[0] > analogRead(SS1)) minADC[0] = analogRead(SS1);
    if (minADC[1] > analogRead(SS2)) minADC[1] = analogRead(SS2);
    if (minADC[2] > analogRead(SS3)) minADC[2] = analogRead(SS3);
    if (minADC[3] > analogRead(SS4)) minADC[3] = analogRead(SS4);
    if (minADC[4] > analogRead(SS5)) minADC[4] = analogRead(SS5);
    if (minADC[5] > analogRead(SS6)) minADC[5] = analogRead(SS6);
  }

  // Calculate average ADC values
  for (int i=0; i<6; i++)
  {
    tempAverageADC = (maxADC[i] + minADC[i]) / 2;
    Serial.print("tempAverageADC ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(tempAverageADC);
    EEPROM.write(i, tempAverageADC);
    delay(5);
  }
}

void setShiftIC()
{
  digitalWrite(ST_CP, LOW);
  shiftOut(DS, SH_CP, LSBFIRST, shiftData);
  digitalWrite(ST_CP, HIGH);
}

void setLED(uint8_t ledPos)
{
  shiftData |= 0b11111100;
  if (ledPos & 0b00100000)    shiftData &= 0b01111111;
  if (ledPos & 0b00010000)    shiftData &= 0b11111011;
  if (ledPos & 0b00001000)    shiftData &= 0b11110111;
  if (ledPos & 0b00000100)    shiftData &= 0b11101111;
  if (ledPos & 0b00000010)    shiftData &= 0b10111111;
  if (ledPos & 0b00000001)    shiftData &= 0b11011111;
  setShiftIC();
}

void setMotor(int mA, int mB)
{
  int pwmA, pwmB;

  // Set direction of motors
  shiftData &=   0b11111100;
  shiftData |=   0b00000001; 
  if (mA < 0)
  {
    shiftData |= 0b00000010;
    mA -= mA;
  }
  if (mB < 0)
  {
    shiftData &= 0b11111110;
    mB -= mB;
  }
  setShiftIC();

  // Change percent into uint8 type and set PWM signal
  pwmA = (mA * 255)/100;
  pwmB = 255 - (mB * 255)/100;
  analogWrite(PWMA, pwmA);
  analogWrite(PWMB, pwmB);
}

uint8_t getSensor(uint8_t amount)
{
  uint8_t data = 0b0;
  if (amount & 0b100000) 
  {
    Serial.print("SS1: ");
    Serial.println(analogRead(SS1));
    if (analogRead(SS1)>averageADC[0])
      data |= 0b100000;
  }
  if (amount & 0b010000) 
  {
    Serial.print("SS2: ");
    Serial.println(analogRead(SS2));
    if (analogRead(SS2)>averageADC[1])
      data |= 0b010000;
  }
  if (amount & 0b001000) 
  {
    Serial.print("SS3: ");
    Serial.println(analogRead(SS3));
    if (analogRead(SS3)>averageADC[2])
      data |= 0b001000;
  }
  if (amount & 0b000100) 
  {
    Serial.print("SS4: ");
    Serial.println(analogRead(SS4));
    if (analogRead(SS4)>averageADC[3])
      data |= 0b000100;
  }
  if (amount & 0b000010) 
  {
    Serial.print("SS5: ");
    Serial.println(analogRead(SS5));
    if (analogRead(SS5)>averageADC[4])
      data |= 0b000010;
  }
  if (amount & 0b000001) 
  {
    Serial.print("SS6: ");
    Serial.println(analogRead(SS6));
    if (analogRead(SS6)>averageADC[5])
      data |= 0b000001;
  }
  setLED(data);
  return data;
}

void turn90_Left()
{
  int flag = 0, count = 0;
  while (count<RoundOneWheelTurn90Degree - MotorBError)
  {
    if (!digitalRead(B_C1))
    {
      flag = 0;
    }
    else if (flag == 0)
    {
      count++;
      flag = 1;
    }
    setMotor(0,100);
  }
  setMotor(0,0);
}

void turn90_Right()
{
  int flag = 0, count = 0;
  while (count<RoundOneWheelTurn90Degree - MotorAError)
  {
    if (!digitalRead(A_C1))
    {
      flag = 0;
    }
    else if (flag == 0)
    {
      count++;
      flag = 1;
    }
    setMotor(100,0);
  }
  setMotor(0,0);
}
