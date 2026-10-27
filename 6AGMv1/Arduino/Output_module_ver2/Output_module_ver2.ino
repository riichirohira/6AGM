/*
2023/09/10 
  This script is the same as Output_module. 
  The only differences are:
    1. The arduino waits for the ADC conversion to complete.
    2. Serial.begin is removed
*/
int volt;
float T = 10000;
float t = 100;
float R = 990;
float r = 0;
float A = (R * t - r * T) / (T - t);
float B = T * t * (R - r) / (T - t);
int pulseinterval = T;
float pulseintervalfloat;
int initial_loop = 0;
volatile uint8_t interval_update = 0;
// This is set to uint8_t, which occupies one byte, since updating the value of multi-byte register in ISR would require atomic access in the main loop ()

void setup() {
  /* Digital input and output */
  DDRB |= (1 << 1);
  // pin 9 is for pulse output (fast pwm)
  DDRD &= ~((1 << 4) | (1 << 3));
  // pin 4 is ENA input from usb
  // pin 3 is for PCINT

  /* ADC setup */
  ADMUX = ADMUX & B11110000;
  //setting pin A0 as the input. This code does not do anything because all the values are set 0 by default
  ADMUX = ADMUX & B00111111;
  // setting the voltage reference to external (from AREF pin)
  // The 2 codes above are not strictly necessary since the initial value of ADMUX is B00000000

  /* Pin Change Interrupt setup */
  PCICR |= (1 << 2);
  // selecting PORTD
  PCMSK2 |= (1 << 3);
  // selecting pin 3
}

void loop() {
  if ((PIND >> 4) & 1 == 1) {
    if (initial_loop == 0) {
      initial_loop = 1;
      TCCR1A &= B00001111;
      TCCR1A |= ((1 << 6) | (1 << 1) | 1);
      TCCR1B &= B11111010;  //Prescaler is initially set to 64 so clear it
      TCCR1B |= ((1 << 4) | (1 << 3) | (1 << 1));
      OCR1A = T; //CHANGED
      // Toggle OC1A on compare match, mode 15 (Fast PWM)
      // Prescaler set to 8
    }
    if (initial_loop == 1) {
      if (interval_update == 1) {
        while (ADCSRA & (1 << ADSC) == 1) {}
        volt = ADCL | (ADCH << 8);
        // No need for atomic read, since the ISR does not update any multi-byte registers
        pulseintervalfloat = B / (volt + A);
        pulseinterval = round(pulseintervalfloat);
        OCR1A = pulseinterval - 1;
        // Same here, no need for atomic write
        interval_update = 0;
      }
    }
  } else {
    initial_loop = 0;
    TCCR1A &= ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4));
  }
}

ISR(PCINT2_vect) {
  ADCSRA |= ((1 << 7) | (1 << 6));
  interval_update = 1;
}
