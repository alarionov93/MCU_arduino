#include <LiquidCrystal.h>

#include <EEPROM.h>
#include <OneWire.h>
//#include <SoftwareSerial.h>
#include "HD44780CustomChars.h"

#define ONE_WIRE_BUS        11
#define NUM_SENSORS         2

// #define SS_RX               12
// #define SS_TX               13
#define HIGER_TEMP          100 // TODO: watch in Transalp specification !!
#define SIG_PIN             7
#define BTN_DEC             8 // decrease led light on LCD
#define LCD_LED             10
#define STATUS_LED_PIN      12
#define EEPROM_BRIGHT_ADDR  0
#define ERR_LCD_IDX         14 //index of err message on lcd

OneWire  ds(ONE_WIRE_BUS);  // on pin 11
// SoftwareSerial SoftSerial(SS_RX, SS_TX); // RX, TX
LiquidCrystal lcd(9,6,5,4,3,2);
/* ERROR CODES:
0 - problem with 1wire CRC
1 - problem with 1wire DEVICES ADDRESSES
2 - low pressure
3 - gps bat get level
4 - high voltage
5 - high temperature
 */

/* TODO: set up status variable and write 1 on STATUS_LED_PIN if alright, and 0 if NOT !!
 Thus, every place with error should turn off status led */

int brightness;
char buff[50]="";
volatile int mode = 0;
volatile int percent = 0;
uint8_t bat_character;
uint8_t chg_symbol = 0;
int countP = 0;
int addr_counter = 0; /* Necessary for determining exactly which (by count) sensor on 1wire bus is sending data in every iteration.
(Because typically this code can get data only from 1 sensor on bus in every iteration of loop() function)
1wire NUM_SENSORS constant SHOULD be defined properly in order to achieve quality of printing temperatures in different LCD places.*/


// TODO: replace all vars of BYTE type to uint8_t and NEVER USE BYTE anymore!!!

void pciSetup(byte pin)
{
    // PCMSK - enable pin in interrupt mask register
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
    // clear any outstanding interrupt
    // PCIFR do not understand if this is necessary
    // (if questions - m328p datasheet, page 93)
    PCIFR  |= bit (digitalPinToPCICRbit(pin));
    // enable interrupt for the group
    // PCICR should nave 0 and 1 bits set (PCIE0 and PCIE1) to enable interrupts
    // for pins [0:7] and [8:14] (as defined in m328p datasheet, page 92)
    PCICR  |= bit (digitalPinToPCICRbit(pin));
}
 
// Use one Routine to handle each group
 
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
    // brightness -
    noInterrupts();
    if( (PINB & (1 << PINB0)) == 1 )
    {
      /* LOW to HIGH pin change */
    }
    else
    {
      /* HIGH to LOW pin change */
      if (brightness >= 0)
      {
        brightness = brightness - 25;
        if (brightness < 0) 
        {
          brightness = 250;
        }
        analogWrite(LCD_LED, brightness);
        EEPROM.update(EEPROM_BRIGHT_ADDR, brightness);
      }
    }
    interrupts();
}
 
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
    noInterrupts();
    digitalWrite(STATUS_LED_PIN, HIGH); // if previous recieving was with error
    // reset data
    memset(buff, '\0', 50);
    
    uint8_t data;
    int i = 0;

    volatile char * pch;
    volatile char mode_str[4]="";
    volatile char percent_str[4]="";
    
    while (Serial.available() > 0) {
        data = Serial.read();
        buff[i] = data;
        i++;
    }
    // Ohhh, got it to work. If there will 
    // be no sign of data inside ISR, 
    // call lcd.print() right HERE !
    if (strstr(buff, "+CBC") != NULL) {
        lcd.setCursor(ERR_LCD_IDX,0); // error 3 is gone
        lcd.print("  ");
        pch = strtok(buff, ":");
        strcpy(mode_str, strtok(NULL, ","));
        strcpy(percent_str, strtok(NULL, ","));
//        strcpy(voltage, strtok(NULL, "\r"));
        mode = atoi(mode_str);
        percent = atoi(percent_str);

        digitalWrite(13, HIGH);
    } else {
      //Serial.println("CH=NO_DATA_1;");
      /* TODO: say about problem on lcd */
      digitalWrite(STATUS_LED_PIN, LOW); // error 3
      // lcd.setCursor(ERR_LCD_IDX,0);
      // lcd.print("#");
      // lcd.print(3); // TODO: causes an error with empty lcd, FIX IT!@
    }
    interrupts();
}

/* TODO: #include LCD lib and define lcd variable */
void print_custom_char(int idx, int col, int row, uint8_t representation) {
  lcd.createChar(idx, representation);
  lcd.setCursor(col, row);
  lcd.write(idx);
}

void setup(void) {

  Serial.begin(19200);
  // SoftSerial.begin(19200);
  lcd.begin(16,2);

  pinMode(SIG_PIN, INPUT);
  pinMode(BTN_DEC, INPUT);
  pinMode(LCD_LED, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH); //show that everything is alright

  // get brightness stored in EEPROM
  noInterrupts();
  brightness = EEPROM.read(EEPROM_BRIGHT_ADDR);
  
  //set initial brightness of lcd
  analogWrite(LCD_LED, brightness);
  
  // enable interrupts by buttons
  pciSetup(SIG_PIN);
  pciSetup(BTN_DEC);
  interrupts();
}

void loop(void) {
  digitalWrite(13, LOW);
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  uint8_t prev_addr[8];
  uint8_t addresses[5][8];
  float celsius, fahrenheit;
  
  // 1wire controls here

  if ( !ds.search(addr)) {
    Serial.println("NA;");
    // delay is needed because BT module cant work so fast and is not sending ALL the data
    delay(200);
//    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      /* TODO: say about problem on lcd and status led */
      Serial.println("CRC_NOT_VALID;");
      digitalWrite(STATUS_LED_PIN, LOW);
      lcd.setCursor(ERR_LCD_IDX,0);// error 0
      lcd.write('#');
      lcd.write('0');
      delay(200);
      return;
  } else {
      lcd.setCursor(ERR_LCD_IDX,0); // error 0 is gone
      lcd.print("  ");
  }
//  Serial.println();

 
  // the first ROM byte indicates which chip
  
  switch (addr[0]) {
    case 0x10:
//      Serial.println("  Chip = DS18S20");  // or old DS1820
      lcd.setCursor(ERR_LCD_IDX,0); // error 1 is gone
      lcd.print("  ");
      type_s = 1;
      break;
    case 0x28:
//      Serial.println("  Chip = DS18B20");
      lcd.setCursor(ERR_LCD_IDX,0); // error 1 is gone
      lcd.print("  ");
      type_s = 0;
      break;
    case 0x22:
//      Serial.println("  Chip = DS1822");
      lcd.setCursor(ERR_LCD_IDX,0); // error 1 is gone
      lcd.print("  ");
      type_s = 0;
      break;
    default:
      /* TODO: say about problem on lcd and status led*/
      Serial.println("DEVICE_ERROR;");
      digitalWrite(STATUS_LED_PIN, LOW); // error 1
      lcd.setCursor(ERR_LCD_IDX,0);
      lcd.write('#');
      lcd.write('1');
      delay(200);
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  int cel = (int) celsius; //int value to print on LCD !
//  fahrenheit = celsius * 1.8 + 32.0;
  // Serial.print("T=");
  // Serial.print(celsius);
  // Serial.print("C;");
  // Serial.println();
  Serial.println(mode);
  Serial.println(percent);
  /* TODO: ! ONLY if other methods will not work !
  in different iterations of loop check the addr var
  and print to different places on lcd by addr value */
//  if (prev_addr) {
//
//  }
  // prev_addr = addr;
//  strcpy(addresses[addr_counter], addr);
  
  // if(addr_counter > 1) {
  //   if (addr == addresses[0]) {
  //     Serial.println("1st;");
  //   } else if (addr == addresses[1]) {
  //     Serial.println("2nd;");
  //   }
  // }
  // Serial.println("*ADDR*");
  // for (int k = 0; k < 8; k++)
  // {
  //   Serial.print(addr[k], HEX);
  //   addresses[addr_counter][k] = addr[k];
  // }
  // Serial.println("*ADDR_END*");
  // Serial.println(addr_counter);
  // Serial.println("*ADDR_ARR*");
  // for (int k = 0; k < 8; k++)
  // {
  //  Serial.print(addresses[addr_counter][k], HEX);
  // }
  // Serial.println("*ADDR_ARR_END*");
//  bool stAddr = false;
//  bool ndAddr = false;
//  for (int j = 0; j < 8; j++) {
//     if (addr[j] != addresses[0][j]) {
//        break;
//     } else {
//        stAddr = true;
//     }
//  }
//  for (int j = 0; j < 8; j++) {   
//     if (addr[j] != addresses[1][j]) {
//        break;
//     } else {
//        ndAddr = true;
//     }
//  }
//  if (stAddr == true) {
//    Serial.println("1st;");
//  }
//  else if (ndAddr == true) {
//    Serial.println("2nd;");
//  } else {
//    
//  }

  if (addr_counter == 0) 
  {
//    Serial.println("1st;");
    // Print 1st temperature to LCD
    Serial.print("T1=");
    Serial.print(celsius);
    Serial.print("C;");
    Serial.println();

    lcd.setCursor(0,0);
    lcd.print("     "); // clear the place for temperature
    lcd.print(cel);
    if (cel > HIGER_TEMP) 
    {
      lcd.print("C!");
      lcd.setCursor(ERR_LCD_IDX,0); //error 5
      lcd.print("#");
      lcd.print(5);
    }
    else 
    {
      lcd.setCursor(ERR_LCD_IDX,0);
      lcd.print("  "); // error is gone
      lcd.print("C ");
    }
  }
  else if (addr_counter == 1)
  {
//    Serial.println("2nd;");
    // Print 2nd temperature to LCD
    Serial.print("T2=");
    Serial.print(celsius);
    Serial.print("C;");
    Serial.println();
    lcd.setCursor(0,1);
    lcd.print(cel);
    lcd.print("C ");
  } else {

  }
  
  addr_counter++;
  if (addr_counter > NUM_SENSORS - 1)
  {
    addr_counter = 0;
  }
  delay(200);

  // get data from analog sensors here
  int pressureSensorValue = analogRead(A0);
  //  int fuelSensorValue = analogRead(A1);
  int voltageSensorValue = analogRead(A2);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltageValue = voltageSensorValue * (5.0 / 1023.0);
  float pureVoltage = voltageValue * 4;
  // pressure conversion here
  float pressureValue = pressureSensorValue * (150.0 / 1023.0);
  float purePressure;
  float middlePoint = 819/2;// because 0.5v - 0psi, but 102 after ADC
  //and 4.5v is 150psi but 921 after ADC prescaling, so full range is 819
  lcd.setCursor(ERR_LCD_IDX,0); // error 2 is gone
  lcd.print("  ");
  if (pressureSensorValue > 923 && pressureSensorValue <= 1023) {
    purePressure = 150.0;
  } else if (pressureSensorValue > 102 && pressureSensorValue <= 923) {
    if (pressureSensorValue >= middlePoint) {
      purePressure = pressureValue + pressureValue * 0.11;
    } else if (pressureSensorValue < middlePoint) {
      purePressure = pressureValue - pressureValue * 0.11;
    }
  } else {
    /* TODO: say about problem on lcd */
    /* TODO: in this case if pressure is LOWER than listed in service manual */
    lcd.setCursor(ERR_LCD_IDX,0); // error 2
    lcd.write('#');
    lcd.write('2');
    purePressure = 0.0;

  }
  if (addr_counter == 0) { // show data only in one of cycles (because of 1wire sensors, which send data only from one sensor in 1 iteration of loop)
    Serial.print("P=");
    Serial.print(purePressure);
    Serial.print("psi;");
    Serial.println();
    lcd.setCursor(5,0);
    lcd.print((int)purePressure);
    lcd.print("p ");
    delay(200);
    // Voltage - print here
    Serial.print("V=");
    Serial.print(pureVoltage);
    Serial.print("V;");
    Serial.println();
    lcd.setCursor(5,1);
    lcd.print(pureVoltage);
    lcd.print("V");
    delay(200);
    // Print all data to LCD below
    // TODO: replace Serial.print with print_custom_char !!

    // GPS tracker battery info interpretating below
    // this section should be in ISR. This is fucking shit, but the only solution I have:-/
//        if (mode == 0 && percent < 50)
//        {
//          /* TODO: should warn about possible errors
//          /* maybe there is trouble with charge circuit or connection */
//          bat_character = is_not_chg_warn;
//          Serial.println("Warning! No data, mode and percent is 0.");
//        }
        // TODO: move the section which lies below AWAY from interrupt, now we have data inside of ISR!!
    
      lcd.createChar(0, bat0_char);
      lcd.createChar(1, bat20_char);
      lcd.createChar(2, bat40_char);
      lcd.createChar(3, bat60_char);
      lcd.createChar(4, bat80_char);
      lcd.createChar(5, bat100_char);
      lcd.createChar(6, is_chg_char);
      lcd.createChar(7, is_not_chg_warn);
      // lcd.setCursor(12,1);
      // lcd.write(6);
      if (mode == 1) {
        uint8_t chg_symbol = is_chg_char;
        Serial.println("CHG=TRUE;");
        // TODO: show chg character on lcd
        lcd.createChar(6, is_chg_char);
        lcd.setCursor(15,1);
        lcd.write(byte(6));
      } else {
        Serial.println("CHG=FALSE;");
        lcd.setCursor(15,1);
        lcd.print(" ");
        // TODO: hide chg character on lcd
      }

      lcd.setCursor(16,1);
    
      // if by ranges of percent value
      if (percent > 0 && percent <= 7)
      {
        /* show 0% charged */
        bat_character = bat0_char;
        Serial.println("CH=7prc;");
        lcd.createChar(0, bat0_char);
        // lcd.print(bat_character);
        lcd.write(byte(0));
      }
      else if (percent > 7 && percent <= 20)
      {
        /* show 20% charged */
        bat_character = bat20_char;
        Serial.println("CH=20prc;");
        lcd.createChar(1, bat20_char);
        lcd.write(byte(1));
      }
      else if (percent > 20 && percent <= 40)
      {
        /* show 40% charged */
        bat_character = bat40_char;
        lcd.createChar(2, bat40_char);
        Serial.println("CH=40prc;");
        lcd.write(byte(2));
      }
      else if (percent > 40 && percent <= 60)
      {
        /* show 60% charged */
        bat_character = bat60_char;
        lcd.createChar(3, bat60_char);
        Serial.println("CH=60prc;");
        lcd.write(byte(3));
      } 
      else if (percent > 60 && percent <= 80)
      {
        /* show 80% charged */
        bat_character = bat80_char;
        Serial.println("CH=80prc;");
        lcd.createChar(4, bat80_char);
        lcd.write(byte(4));
      }
      else if (percent > 80 && percent <= 100)
      {
        /* show 100% charged */
        bat_character = bat100_char;
        Serial.println("CH=100prc;");
        lcd.createChar(5, bat100_char);
        lcd.write(byte(5));
      }
      else
      {
       /* show warn about possible errors */
       // bat_character = is_not_chg_warn;
       Serial.println("CH=NO_DATA_0;");
       lcd.print("?"); // TODO: error 3 here, too
       //show error on status led // how to show error properly (because this code is within interrupt now)
       // digitalWrite(STATUS_LED_PIN, LOW);         
       // TODO: show "!" near bat character on lcd
      }
  } 
}
