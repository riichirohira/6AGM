/* Changes from Rotary_encoder_on_MEGA_ver7_absolute_all_bits
  1. when reading digital input, compare it to 1, not 0
  2. 
*/

volatile int vcount1 = 0;
volatile int vcount2 = 0;
volatile int vcount3 = 0;
volatile int vcount4 = 0;
volatile int vcount5 = 0;
volatile int vcount6 = 0;

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
int count5 = 0;
int count6 = 0;

volatile int vfcount1 = 0;
volatile int vfcount2 = 0;
volatile int vfcount3 = 0;
volatile int vfcount4 = 0;
volatile int vfcount5 = 0;
volatile int vfcount6 = 0;

int fcount1 = 0;
int fcount2 = 0;
int fcount3 = 0;
int fcount4 = 0;
int fcount5 = 0;
int fcount6 = 0;

// volatile int vinitialz1 = 0;
// volatile int vinitialz2 = 0;
// volatile int vinitialz3 = 0;
// volatile int vinitialz4 = 0;
// volatile int vinitialz5 = 0;
// volatile int vinitialz6 = 0;

int initialz1 = 0;
int initialz2 = 0;
int initialz3 = 0;
int initialz4 = 0;
int initialz5 = 0;
int initialz6 = 0;

volatile uint8_t flag1 = 0;
volatile uint8_t flag2 = 0;
volatile uint8_t flag3 = 0;
volatile uint8_t flag4 = 0;
volatile uint8_t flag5 = 0;
volatile uint8_t flag6 = 0;

// volatile uint8_t zdone1 = 0;
// volatile uint8_t zdone2 = 0;
// volatile uint8_t zdone3 = 0;
// volatile uint8_t zdone4 = 0;
// volatile uint8_t zdone5 = 0;
// volatile uint8_t zdone6 = 0;

uint8_t zdone1 = 0;
uint8_t zdone2 = 0;
uint8_t zdone3 = 0;
uint8_t zdone4 = 0;
uint8_t zdone5 = 0;
uint8_t zdone6 = 0;

int remain1 = 0;
int remain2 = 0;
int remain3 = 0;
int remain4 = 0;
int remain5 = 0;
int remain6 = 0;

int prevcopy1 = 0;
int prevcopy2 = 0;
int prevcopy3 = 0;
int prevcopy4 = 0;
int prevcopy5 = 0;
int prevcopy6 = 0;

int curcopy1 = 0;
int curcopy2 = 0;
int curcopy3 = 0;
int curcopy4 = 0;
int curcopy5 = 0;
int curcopy6 = 0;

uint8_t H_count1 = 0;
uint8_t H_count2 = 0;
uint8_t H_count3 = 0;
uint8_t H_count4 = 0;
uint8_t H_count5 = 0;
uint8_t H_count6 = 0;
uint8_t M_count1 = 0;
uint8_t M_count2 = 0;
uint8_t M_count3 = 0;
uint8_t M_count4 = 0;
uint8_t M_count5 = 0;
uint8_t M_count6 = 0;
uint8_t L_count1 = 0;
uint8_t L_count2 = 0;
uint8_t L_count3 = 0;
uint8_t L_count4 = 0;
uint8_t L_count5 = 0;
uint8_t L_count6 = 0;

void setup() {
  /* Serial setup */
  unsigned long BaudRate = 115200;
  unsigned long F_OSC = 16000000;
  unsigned int UBRR_value = F_OSC / 8 / BaudRate - 1;


  // Double the USART transmission speed
  UCSR0A |= (1 << U2X0);


  // Initialize Baud Rate
  // UBRR0H = (UBRR_value << 8); This is the previous version
  UBRR0H = (UBRR_value >> 8);
  UBRR0L = UBRR_value;


  // USART mode selection --> Asynchronous
  UCSR0C &= ~((1 << UMSEL01) | (1 << UMSEL00));


  // Parity mode selection --> Disable
  UCSR0C &= ~((1 << UPM01) | (1 << UPM00));


  // Stop bit selection --> 1-bit
  UCSR0C &= ~(1 << USBS0);


  // Select the number of data bits --> 8 bits
  UCSR0B &= ~(1 << UCSZ02);
  UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));


  // Enable USART transmitter
  UCSR0B |= (1 << TXEN0);



  /* (For EXTINT) Set PORTD and PORTE as INPUT, and PULL-UP */
  /* (For B and Z phase output) Set PORTA and PORTL as INPUT, and PULL-UP */
  DDRD &= 0;
  DDRE &= 0;
  DDRA &= 0;
  DDRL &= 0;
  PORTD |= 0xFF;
  PORTE |= 0xFF;
  PORTA |= 0xFF;
  PORTL |= 0xFF;

  /* Set EXTINT (INT0:5) */
  EIMSK &= 0;                       // Disable interrupt
  EICRA &= 0;                       // Initialize register
  EICRB &= 0;                       // Initialize register
  EICRA |= 0xFF;                    // all bits to 0 --> Interrupt on the RISING edge for INT 0:3
  EICRB |= 0xF;                     // 4 bits to 0 --> Interrupt on the RISING edge for INT 4:5
  EIFR |= ~((1 << 6) | (1 << 7));   // clear any flags that might have occured to avoid executing any pending EXTINTs
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
}

void loop() {
  /* Motor 1 */
  EIMSK &= 0;                       // turn off interrupts
  count1 = vcount1;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag1 == 1) {
    flag1 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount1 = vfcount1;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone1 == 0) {
      zdone1 = 1;
      initialz1 = fcount1;
    }
    prevcopy1 = fcount1;
    remain1 = (count1 - initialz1) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain1 = (remain1 >= 0) ? remain1 : remain1 + 500;                                    // 0 <= remain < 500
    count1 = (remain1 >= 0 && remain1 < 250) ? count1 - remain1 : count1 + 500 - remain1;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy1 = vcount1;                       //copy the current vcount
    vcount1 = curcopy1 - prevcopy1 + count1;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count1 = (count1 >> 14 & 0xF);            // 4 ones
  M_count1 = count1 >> 7 & 0x7F;              // 7 ones
  L_count1 = count1 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (1 << 4) | H_count1);
  Serial.write(M_count1);
  Serial.write(L_count1);

  /* Motor 2 */
  EIMSK &= 0;                       // turn off interrupts
  count2 = vcount2;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag2 == 1) {                 // Rigorously speaking, the flag needs to be checked before the interrupt is re-enabled in order to ensure that the vcount we just referenced was obtained just as the flag was set (practically, such a fluctuation is easily compensated by the code below)
    flag2 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount2 = vfcount2;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone2 == 0) {
      zdone2 = 1;
      initialz2 = fcount2;
    }
    prevcopy2 = fcount2;
    remain2 = (count2 - initialz2) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain2 = (remain2 >= 0) ? remain2 : remain2 + 500;                                    // 0 <= remain < 500
    count2 = (remain2 >= 0 && remain2 < 250) ? count2 - remain2 : count2 + 500 - remain2;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy2 = vcount2;                       // copy the current vcount
    vcount2 = curcopy2 - prevcopy2 + count2;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count2 = (count2 >> 14 & 0xF);            // 4 ones
  M_count2 = count2 >> 7 & 0x7F;              // 7 ones
  L_count2 = count2 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (2 << 4) | H_count2);
  Serial.write(M_count2);
  Serial.write(L_count2);

  /* Motor 3 */
  EIMSK &= 0;                       // turn off interrupts
  count3 = vcount3;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag3 == 1) {                 // Rigorously speaking, the flag needs to be checked before the interrupt is re-enabled in order to ensure that the vcount we just referenced was obtained just as the flag was set (practically, such a fluctuation is easily compensated by the code below)
    flag3 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount3 = vfcount3;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone3 == 0) {
      zdone3 = 1;
      initialz3 = fcount3;
    }
    prevcopy3 = fcount3;
    remain3 = (count3 - initialz3) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain3 = (remain3 >= 0) ? remain3 : remain3 + 500;                                    // 0 <= remain < 500
    count3 = (remain3 >= 0 && remain3 < 250) ? count3 - remain3 : count3 + 500 - remain3;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy3 = vcount3;                       // copy the current vcount
    vcount3 = curcopy3 - prevcopy3 + count3;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count3 = (count3 >> 14 & 0xF);            // 4 ones
  M_count3 = count3 >> 7 & 0x7F;              // 7 ones
  L_count3 = count3 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (3 << 4) | H_count3);
  Serial.write(M_count3);
  Serial.write(L_count3);

  /* Motor 4 */
  EIMSK &= 0;                       // turn off interrupts
  count4 = vcount4;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag4 == 1) {
    flag4 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount4 = vfcount4;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone4 == 0) {
      zdone4 = 1;
      initialz4 = fcount4;
    }
    prevcopy4 = fcount4;
    remain4 = (count4 - initialz4) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain4 = (remain4 >= 0) ? remain4 : remain4 + 500;                                    // 0 <= remain < 500
    count4 = (remain4 >= 0 && remain4 < 250) ? count4 - remain4 : count4 + 500 - remain4;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy4 = vcount4;                       //copy the current vcount
    vcount4 = curcopy4 - prevcopy4 + count4;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count4 = (count4 >> 14 & 0xF);            // 4 ones
  M_count4 = count4 >> 7 & 0x7F;              // 7 ones
  L_count4 = count4 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (4 << 4) | H_count4);
  Serial.write(M_count4);
  Serial.write(L_count4);

  /* Motor 5 */
  EIMSK &= 0;                       // turn off interrupts
  count5 = vcount5;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag5 == 1) {
    flag5 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount5 = vfcount5;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone5 == 0) {
      zdone5 = 1;
      initialz5 = fcount5;
    }
    prevcopy5 = fcount5;
    remain5 = (count5 - initialz5) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain5 = (remain5 >= 0) ? remain5 : remain5 + 500;                                    // 0 <= remain < 500
    count5 = (remain5 >= 0 && remain5 < 250) ? count5 - remain5 : count5 + 500 - remain5;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy5 = vcount5;                       //copy the current vcount
    vcount5 = curcopy5 - prevcopy5 + count5;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count5 = (count5 >> 14 & 0xF);            // 4 ones
  M_count5 = count5 >> 7 & 0x7F;              // 7 ones
  L_count5 = count5 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (5 << 4) | H_count5);
  Serial.write(M_count5);
  Serial.write(L_count5);

  /* Motor 6 */
  EIMSK &= 0;                       // turn off interrupts
  count6 = vcount6;                 // copy value
  EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
  if (flag6 == 1) {
    flag6 = 0;
    EIMSK &= 0;  // Disable interrupts
    fcount6 = vfcount6;
    EIMSK |= ~((1 << 6) | (1 << 7));  // Re-enable interuupt
    if (zdone6 == 0) {
      zdone6 = 1;
      initialz6 = fcount6;
    }
    prevcopy6 = fcount6;
    remain6 = (count6 - initialz6) % 500;                                                  // Ideally, count value when the flag is up minus initialz is multiple of 500
    remain6 = (remain6 >= 0) ? remain6 : remain6 + 500;                                    // 0 <= remain < 500
    count6 = (remain6 >= 0 && remain6 < 250) ? count6 - remain6 : count6 + 500 - remain6;  // redefine count (the corrected value of the previous vcount)
    /*
    The value of vcount would have changed from the last time we referenced it, so the corrected value of vcount is:
   (the corrected value of vcount) = (the current value of vcount) - (the original value of the previous vcount) + (the corrected value of the previous vcount)
    (assuming no interrupts were missed in the meantime, and even if some were missed, the effects are negiligible)
    */
    EIMSK &= 0;                               // turn off interrupts
    curcopy6 = vcount6;                       //copy the current vcount
    vcount6 = curcopy6 - prevcopy6 + count6;  //
    EIMSK |= ~((1 << 6) | (1 << 7));          // Re-enable interuupt
  }                                           //
  H_count6 = (count6 >> 14 & 0xF);            // 4 ones
  M_count6 = count6 >> 7 & 0x7F;              // 7 ones
  L_count6 = count6 & 0x7F;                   // 7 ones
  Serial.write((1 << 7) | (6 << 4) | H_count6);
  Serial.write(M_count6);
  Serial.write(L_count6);
}

/* Motor 1 */
ISR(INT1_vect) {
  if ((PINA >> 1) & 1 == 1) {
    vcount1--;
    // if ((PINL >> 1) & 1 == 1) {
    //   rot_count1A++;
    // }
  } else {
    vcount1++;
    // if ((PINL >> 1) & 1 == 1) {
    //   rot_count1A--;
    // }
  }
  if ((PINL >> 1) & 1 == 1) {
    // flag1 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag1 = 1;
    vfcount1 = vcount1;
    // if (zdone1 == 0) {
    //   vinitialz1 = vcount1;
    //   zdone1 = 1;
    // }
  }
}
/* Motor 2 */
ISR(INT2_vect) {
  if ((PINA >> 2) & 1 == 1) {
    vcount2--;
    // if ((PINL >> 2) & 1 == 1) {
    //   rot_count1B++;
    // }
  } else {
    vcount2++;
    // if ((PINL >> 2) & 1 == 1) {
    //   rot_count1B--;
    // }
  }
  if ((PINL >> 2) & 1 == 1) {
    // flag2 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag2 = 1;
    vfcount2 = vcount2;
    // if (zdone2 == 0) {
    //   vinitialz2 = vcount2;
    //   zdone2 = 1;
    // }
  }
}
/* Motor 3 */
ISR(INT3_vect) {
  if ((PINA >> 3) & 1 == 1) {
    vcount3--;
    // if ((PINL >> 3) & 1 == 1) {
    //   rot_count1C++;
    // }
  } else {
    vcount3++;
    // if ((PINL >> 3) & 1 == 1) {
    //   rot_count1C--;
    // }
  }
  if ((PINL >> 3) & 1 == 1) {
    // flag3 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag3 = 1;
    vfcount3 = vcount3;
    // if (zdone3 == 0) {
    //   vinitialz3 = vcount3;
    //   zdone3 = 1;
    // }
  }
}
/* Motor 4 */
ISR(INT4_vect) {
  if ((PINA >> 4) & 1 == 1) {
    vcount4--;
    // if ((PINL >> 4) & 1 == 1) {
    //   rot_count4A++;
    // }
  } else {
    vcount4++;
    // if ((PINL >> 4) & 1 == 1) {
    //   rot_count4A--;
    // }
  }
  if ((PINL >> 4) & 1 == 1) {
    // flag4 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag4 = 1;
    vfcount4 = vcount4;
    // if (zdone4 == 0) {
    //   vinitialz4 = vcount4;
    //   zdone4 = 1;
    // }
  }
}
/* Motor 5 */
ISR(INT5_vect) {
  if ((PINA >> 5) & 1 == 1) {
    vcount5--;
    // if ((PINL >> 5) & 1 == 1) {
    //   rot_count4B++;
    // }
  } else {
    vcount5++;
    // if ((PINL >> 5) & 1 == 1) {
    //   rot_count4B--;
    // }
  }
  if ((PINL >> 5) & 1 == 1) {
    // flag5 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag5 = 1;
    vfcount5 = vcount5;
    // if (zdone5 == 0) {
    //   vinitialz5 = vcount5;
    //   zdone5 = 1;
    // }
  }
}
/* Motor 6 */
ISR(INT0_vect) {
  if ((PINA >> 6) & 1 == 1) {
    vcount6--;
    // if ((PINL >> 6) & 1 == 1) {
    //   rot_count4C++;
    // }
  } else {
    vcount6++;
    // if ((PINL >> 6) & 1 == 1) {
    //   rot_count4C--;
    // }
  }
  if ((PINL >> 6) & 1 == 1) {
    // flag6 = 0;  // the flag should be set if and only if the A phase and the Z phase are HIGH
  } else {
    flag6 = 1;
    vfcount6 = vcount6;
    // if (zdone6 == 0) {
    //   vinitialz6 = vcount6;
    //   zdone6 = 1;
    // }
  }
}