#include <EEPROM.h>
#include <OneWire.h>
// #include <SoftwareSerial.h>
#include "HD44780CustomChars.h"

#define ONE_WIRE_BUS        11
// #define SS_RX               12
// #define SS_TX               13
#define SIG_PIN             7
#define BTN_DEC             8 //decrease led light on LCD
#define LCD_LED             10
#define EEPROM_BRIGHT_ADDR  0

OneWire  ds(ONE_WIRE_BUS);  // on pin 11 (a 4.7K resistor is necessary)
// SoftwareSerial SoftSerial(SS_RX, SS_TX); // RX, TX

int brightness;
char buff[50]="";
int mode = 0;
int percent = 0;

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
    cli();
    if( (PINB & (1 << PINB0)) == 1 )
    {
      /* LOW to HIGH pin change */
    }
    else
    {
      /* HIGH to LOW pin change */
      if (brightness > 0)
      {
        brightness = brightness - 25;
        analogWrite(LCD_LED, brightness);
        EEPROM.update(EEPROM_BRIGHT_ADDR, brightness);
      }
    }
    // TODO: verify if this code below is good and compileable
    if ( (PINB & (1 << PINB1)) == 1 )
    {
      // then we recieved data about gps tracker battery status and charge
    }
    sei();
    reti();
}
 
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
    cli();
    char * pch;
    char mode_str[4]="";
    char percent_str[4]="";
    // reset data
    mode = 0;
    percent = 0;
    memset(buff, '\0', 50);

    if (Serial.available() > 0) {
        Serial.readBytes(buff, 40);
    }
    if (strstr(buff, "+CBC") != NULL) {
        pch = strtok(buff, ":");
        strcpy(mode, strtok(NULL, ","));
        strcpy(percent, strtok(NULL, ","));
        mode = atoi(mode);
        percent = atoi(percent);
        Serial.println("Chg lvl recieved.");
        Serial.println(mode);
        Serial.println(percent);
    }
    sei();
    reti();
}

/* TODO: #include LCD lib and define lcd variable */
// void print_custom_char(int idx, int col, int row, uint8_t representation) {
//   lcd.createChar(idx, representation);
//   lcd.setCursor(col, row);
//   lcd.write(idx);
// }

void setup(void) {

  Serial.begin(19200);
  // SoftSerial.begin(19200);

  pinMode(SIG_PIN, INPUT);
  pinMode(BTN_DEC, INPUT);
  pinMode(LCD_LED, OUTPUT);

  // get brightness stored in EEPROM
  brightness = EEPROM.read(EEPROM_BRIGHT_ADDR);
  //set initial brightness of lcd
  analogWrite(LCD_LED, brightness);
  // enable interrupts by buttons
  cli();
  pciSetup(SIG_PIN);
  pciSetup(BTN_DEC);
  sei();
}

void loop(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  // 1wire controls here

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
//      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
//      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
//      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
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
//  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("Temperature = ");
  Serial.print(celsius);
  Serial.print(" C");
//  Serial.print(fahrenheit);
  Serial.println();

  // get data from analog sensors here
  int sensorValue = analogRead(A0);
//  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float pressureValue = sensorValue * (100.0 / 1023.0);
  float purePressure;
  float middlePoint = 819/2;// because 0.5v - 0psi, but 102 after ADC
  //and 4.5v is 100psi but 921 after ADC prescaling, so full range is 819
  if (sensorValue > 923 && sensorValue <= 1023) {
    purePressure = 100.0;
  } else if (sensorValue > 102 && sensorValue <= 923) {
    if (sensorValue >= middlePoint) {
      purePressure = pressureValue + pressureValue * 0.11;
    } else if (sensorValue < middlePoint) {
      purePressure = pressureValue - pressureValue * 0.11;
    }
  } else {
    purePressure = 0.0;
  }
  Serial.print("Pressure = ");
  Serial.print(purePressure);
  Serial.print(" psi");
  Serial.println();

  // Print all data to LCD below
  // TODO: replace Serial.print with print_custom_char !!

  // battery info interpretating is below
  if (mode == 0 && percent == 0)
  {
    /* TODO: should warn about possible errors
     * maybe there is trouble with charge circuit or connection */
    uint8_t character = is_not_chg_warn;
    Serial.println("Warning! No data, mode and percent is 0.");
  }

  if (mode == 1) {
    uint8_t character = is_chg_char;
    Serial.println(" ***---Charging!---*** ");
  }

  // if by ranges of percent value
  if (percent > 0 && percent <= 7)
  {
    /* show 0% charged */
    uint8_t character = bat0_char;
    Serial.println("Less than 7 percent left");
  }
  else if (percent > 7 && percent <= 20)
  {
    /* show 20% charged */
    uint8_t character = bat20_char;
    Serial.println("20 percent left");
  }
  else if (percent > 20 && percent <= 40)
  {
    /* show 40% charged */
    uint8_t character = bat40_char;
    Serial.println("40 percent left");
  }
  else if (percent > 40 && percent <= 60)
  {
    /* show 60% charged */
    uint8_t character = bat60_char;
    Serial.println("60 percent left");
  } 
  else if (percent > 60 && percent <= 80)
  {
    /* show 80% charged */
    uint8_t character = bat80_char;
    Serial.println("80 percent left");
  }
  else if (percent > 80 && percent <= 100)
  {
    /* show 100% charged */
    uint8_t character = bat100_char;
    Serial.println("100 percent left");
  }
  else
  {
    /* show warn about possible errors */
    uint8_t character = is_not_chg_warn;
    Serial.println("Warning! No data");
  }
  
}
