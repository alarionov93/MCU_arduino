// Custom characters for displaying battery percentage

#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// 0% charged
byte bat0_char[8] = {
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
byte bat20_char[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111
};
 
int i = 0;
 
void setup() {
  // example of custom character usage below:
  // createChar first param is symbol ID, so,
  // we can write(ID) to write symbol
  // TODO: create char with battery state and % of charging ?
  lcd.createChar(0, bat0_char);
  lcd.createChar(1, bat20_char);

  lcd.begin(16, 2);
  // TODO: change output of characters
  for(int n = 0; n < 2; n++)
  {
    lcd.setCursor(n*2,0);
    lcd.write(n);
  }
}
 
void loop() {
 
}