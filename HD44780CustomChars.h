/** Custom chars
 *  These bytes creates custom chars 
 *  to represent battery data for
 *  LCD display build on HD44780 chip
*/
// 0% charged
uint8_t bat0_char[8] = {
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
uint8_t bat20_char[8] = {
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
uint8_t bat40_char[8] = {
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
uint8_t bat60_char[8] = {
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
uint8_t bat80_char[8] = {
  B01110,
  B11011,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
// 100% 
uint8_t bat100_char[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
// is charging char
uint8_t is_chg_char[8] = {
  B00011,
  B00110,
  B01100,
  B11111,
  B11111,
  B00110,
  B01100,
  B11000
};
// is NOT charging char warn
uint8_t is_not_chg_warn[8] = {
  B01110,
  B11011,
  B10101,
  B10101,
  B10101,
  B10001,
  B10101,
  B11111
};
/* Then, later in MDU_firmware: 
  lcd.createChar(0, bat0_char);
  lcd.createChar(1, bat20_char);
  lcd.createChar(2, bat40_char);
  lcd.createChar(3, bat60_char);
  lcd.createChar(4, bat80_char);
  lcd.createChar(5, bat100_char);
  lcd.createChar(6, is_chg_char);

  lcd.begin(16, 2);
  // TODO: change output of characters
  for(int n = 0; n < 8; n++)
  {
    lcd.setCursor(n*2,0);
    lcd.write(n);
  }
*/