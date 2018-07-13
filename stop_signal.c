volatile uint8_t ss_cnt= 0;
volatile uint8_t pwm_val = 60;
volatile uint8_t stop_state = 0;
volatile uint8_t isr_cnt = 0;
volatile uint8_t blink_cnt = 0;
volatile uint8_t blink_n_cnt = 0;

#define STOP_SW_PORT PORTB
#define STOP_SW_DDR DDRB
#define STOP_SW_IN_REG PINB
#define STOP_SW_PIN PB0

#define LIGHTS_SW_PORT PORTB
#define LIGHTS_SW_DDR DDRB
#define LIGHTS_SW_IN_REG PINB
#define LIGHTS_SW_PIN PB2

#define STOP_LED_PORT PORTB
#define STOP_LED_DDR DDRB
#define STOP_LED_PIN PB1

#define STOP_SW_ON ((STOP_SW_IN_REG & _BV(STOP_SW_PIN)) != 0)
#define STOP_SW_OFF !STOP_SW_ON

#define LIGHTS_SW_ON ((LIGHTS_SW_IN_REG & _BV(LIGHTS_SW_PIN)) != 0)
#define LIGHTS_SW_OFF !LIGHTS_SW_ON

void setup() {
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 300;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // no prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  STOP_LED_DDR |= _BV(STOP_LED_PIN);
  LIGHTS_SW_DDR &= ~_BV(LIGHTS_SW_PIN);
  STOP_SW_DDR &= ~_BV(STOP_SW_PIN);
  sei();
}

int main()
{
  setup();
  while(1) 
  {
    // loop();
    asm("nop");
  }
  return 0;
}

void loop() {
  //Serial.println(pwm_val);
  delay(200);
}

ISR(TIMER1_COMPA_vect)
{
  if (isr_cnt++ > 100)
  {
    isr_cnt = 0;
    switch (stop_state) 
    {
      case 0:
        // all off
        
        pwm_val = 0;
        if (STOP_SW_ON)
        {
          // to 1
          stop_state = 1;
        }
        if (LIGHTS_SW_ON)
        {
          // to 4
          stop_state = 4;
        }
        break;
      case 1:
        // stop blink on
        pwm_val = 255;
        if(blink_cnt++ > 30)
        {
          blink_cnt = 0;
          // to 2
          stop_state = 2;
        }
        break;
      case 2:
        // stop blink off
        pwm_val = 0;
        
        if (blink_n_cnt++ > 100 && STOP_SW_ON)
        {
          // to 3
          blink_n_cnt = 0;
          stop_state = 3;
        }
        if (STOP_SW_OFF)
        {
          blink_n_cnt = 0;
          if(LIGHTS_SW_ON)
          {
            // to 4
            stop_state = 4;
          }
          if (LIGHTS_SW_OFF)
          {
            // to 0
            stop_state = 0;
          }
        }
        if(blink_cnt++ > 30)
        {
          blink_cnt = 0;
          // to 1
          stop_state = 1;
        }
        break;
      case 3:
        // stop on
        pwm_val = 255;
        if (STOP_SW_OFF)
        {
          if(LIGHTS_SW_ON)
          {
            // to 4
            stop_state = 4;
          }
          if (LIGHTS_SW_OFF)
          {
            // to 0
            stop_state = 0;
          }
        }
        break;
      case 4:
        // lights on
        pwm_val = 60;
        if (STOP_SW_ON)
        {
          // to 1
          stop_state = 1;
        }
        if (LIGHTS_SW_OFF)
        {
          // to 0
          stop_state = 0;
        }
        break;
    }
    //пыпыра
  }
  
  TCNT1 = 0;
  
  
  if (ss_cnt++ < pwm_val)
  {
    STOP_LED_PORT |= _BV(STOP_LED_PIN);
  } else {
    STOP_LED_PORT &= ~_BV(STOP_LED_PIN);
  }
}
