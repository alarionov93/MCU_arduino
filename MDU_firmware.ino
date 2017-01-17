#include <EEPROM.h>
#include <OneWire.h>
// "Button+" and "Button-" pins are 7,8

OneWire  ds(11);  // on pin 11 (a 4.7K resistor is necessary)
// SoftwareSerial SoftSerial(SS_RX, SS_TX); // RX, TX

int SIG_PIN = 12; // pin for telling gps tracker to send battery percentage (to cause interrupt on gps tracker) and
// available to use in other functions
int buttonInc = 7;
int buttonDec = 8;
int led = 10;
const int EEPROMBrightnessAddr = 0;
int brightness;
// ISROddCount needed because IS routine works 2 times:
//one by pushing button in, other by button pulls back,
//so only odd interrupts need to be executing
int ISROddCount1 = 1;
int ISROddCount2 = 1;

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

// uint8_t getGPSTrackerBatteryPercentage(unsigned int timeout) {
//   char response[100];
//   memset(response, '\0', 100);
//   digitalWrite(SIG_PIN, HIGH);
//   delay(200);
//   digitalWrite(SIG_PIN, LOW);
//   Serial.print("GET_BAT_PER");
//   int i = 0;
//   unsigned long previous = millis();
//   do {
//     if(Serial.available() != 0)
//     {
//       response[i] = Serial.read();
//       i++;
//     }
//   }
//   while ((millis() - previous) < timeout);
//   resp = atoi(response);
//   return resp;
// }
 
// Use one Routine to handle each group
 
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
    // && ISROddCount1 % 2 == 0
    // brightness -
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
        analogWrite(led, brightness);
        EEPROM.update(EEPROMBrightnessAddr, brightness);
        // delay does not work in ISR!!
      }
    }
    // ISROddCount1++;
    // if (ISROddCount1 > 2) {
    //   ISROddCount1 = 1;
    // }
}
 
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
    // && ISROddCount2 % 2 == 0
    // brightness +
    if( (PIND & (1 << PIND7)) == 1 ){}
    else
    {
      if (brightness < 226)
      {
        brightness = brightness + 25;
        analogWrite(led, brightness);
        EEPROM.update(EEPROMBrightnessAddr, brightness);
      }
    }
    // ISROddCount2++;
    // if (ISROddCount2 > 2) {
    //   ISROddCount2 = 1;
    // }
}

void setup(void) {
  Serial.begin(9600);
  pinMode(buttonInc, INPUT);
  pinMode(buttonDec, INPUT);
  pinMode(led, OUTPUT);

  // get brightness stored in EEPROM
  brightness = EEPROM.read(EEPROMBrightnessAddr);
  //set initial brightness of lcd
  analogWrite(led, brightness);
  // enable interrupts by buttons
  pciSetup(buttonInc);
  pciSetup(buttonDec);
  // EICRA external interrupt enable only by falling edge for PCINT0 and INT1
  // EICRA = 0x0A;
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
  
}
