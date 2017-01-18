// Custom characters for displaying battery percentage

#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
 
// 0% charged
byte newChar1[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111
};
// 20%
byte newChar2[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111
};
// 40%
byte newChar2[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
};
// 60
byte newChar2[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111
};
// 80
byte newChar2[8] = {
  B01110,
  B11011,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
// 90
byte newChar2[8] = {
  B01110,
  B11011,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
// 100% 
byte newChar2[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
 
int i = 0;
 
void setup() {
  lcd.createChar(0, newChar1);
  lcd.createChar(1, newChar2);
  // TODO: create other chars
  lcd.begin(16, 2);
  // TODO: change output of characters
  for(int n = 0; n < 8; n++)
  {
    lcd.setCursor(n*2,0);
    lcd.write(n);
  }
}
 
void loop() {
 
}