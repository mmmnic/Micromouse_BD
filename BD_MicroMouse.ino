#include "Driver.h"

/*------------ SETUP ------------ */
void setup() {
  Serial.begin(115200);
  // Setup pin mode
  pinMode(SH_CP,  OUTPUT);
  pinMode(ST_CP,  OUTPUT);
  pinMode(DS,     OUTPUT);
  pinMode(PWMA,   OUTPUT);
  pinMode(PWMB,   OUTPUT);
  pinMode(SDA,    OUTPUT);
  pinMode(SCL,    OUTPUT);
  pinMode(A_C1,   INPUT);
  pinMode(A_C2,   INPUT);
  pinMode(B_C1,   INPUT);
  pinMode(B_C2,   INPUT);
  pinMode(BUT1,   INPUT);
  pinMode(BUT2,   INPUT);
  pinMode(BUT3,   INPUT);
  pinMode(SS1,    INPUT);
  pinMode(SS2,    INPUT);
  pinMode(SS3,    INPUT);
  pinMode(SS4,    INPUT);
  pinMode(SS5,    INPUT);
  pinMode(SS6,    INPUT);

  // Setup peripheral devices
  lcd.begin();
  lcd.backlight();
  setMotor(0, 0);

  while (!isBut(1))
  {
    writeLCD(0, 1, "BUT1 BAT DAU");
    writeLCD(0, 0, "BUT3 learn color");
    if (isBut(3))
    {
      Serial.println("LEARNING");
      learnColor();
    }
    else if (isBut(2))
    {
      // Cấu hình nút nhấn 2 ở đây
      
    }
  }
  lcd.clear();

  // Get data from EEPROM put to
  for (int i = 0; i < 6; i++)
  {
    averageADC[i] = EEPROM.read(i);
    delay(5);
    Serial.print("averageADC ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(averageADC[i]);
  }
}

int mode = 1;
void loop() {
  // Viết code ở đây
  lcd.clear();
  switch (mode) {
    // Mode chạy thẳng
    case 1:
      writeLCD(0, 1, "    MODE 1");
      writeLCD(0, 0, "MODE CHAY THANG");
      setMotor(75,75);
      if (isBut(1))     mode = 1;
      if (isBut(2))     mode = 2;
      if (isBut(3))     mode = 3;
      break;
    // Mode chạy tiến và lùi
    case 2:
      writeLCD(0, 1, "    MODE 2");
      writeLCD(0, 0, "MODE TIEN VA LUI");
      setMotor(75, 75);
      delay (4000);
      setMotor(-75, -75);
      delay (4000);
      if (isBut(1))     mode = 1;
      if (isBut(2))     mode = 2;
      if (isBut(3))     mode = 3;
      break;
      
    // Mode chạy hình vuông  
    case 3:
      writeLCD(0, 1, "    MODE 3");
      writeLCD(0, 0, "MODE HINH VUONG");
      setMotor(75, 75);
      delay(2000);
      setMotor(0,0);
      turn90_Right();
      if (isBut(1))     mode = 1;
      if (isBut(2))     mode = 2;
      if (isBut(3))     mode = 3;
      break;
  }
}
