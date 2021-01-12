
#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26


//*******************************//
//***Encoder & Interrupt Setup***//
//*******************************//

//Print





//LEFT MOTOR//

volatile long e0_count = 0; // used by encoder to count the rotation //used to be
volatile long prev_e0_count = 0; // used by encoder to count the rotation //used to be
volatile bool oldE0_A;  // used by encoder to remember prior state of A
volatile bool oldE0_B;  // used by encoder to remember prior state of B
// Global volatile timestamps to determine
// speed from within the encoder0 ISR.
volatile unsigned long e0_interval;
volatile unsigned long e0_last_time;
// Global volatile to hold the determined
// speed of wheel fixed to encoder0
volatile float e0_speed;

//RIGHT MOTOR//

volatile long e1_count = 0; // used by encoder to count the rotation
volatile long prev_e1_count = 0; // used by encoder to count the rotation
volatile bool oldE1_A;  // used by encoder to remember prior state of A
volatile bool oldE1_B;  // used by encoder to remember prior state of B
// Global volatile timestamps to determine
// speed from within the encoder0 ISR.
volatile unsigned long e1_interval;
volatile unsigned long e1_last_time;
// Global volatile to hold the determined
// speed of wheel fixed to encoder0
volatile float e1_speed;

//TIMER//

// Global variable to remember the
// on/off state of the LED.
volatile boolean DEBUG_LED_STATE = false;
//Speed calculation refresh frequency
volatile int timer_freq_hz = 20;


//************************//
//***Interrupt Services***//
//************************//

//RIGHT MOTOR//

// This ISR handles just Encoder 1
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect ) {
  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean newE1_B = digitalRead( E1_B_PIN );
  boolean newE1_A = digitalRead( E1_A_PIN );

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE1_A ^= newE1_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );

  int dir = 0; //1 forwards (cw),-1 backwards (ccw)
  if ( state == 1 ) {
    dir = -1; // cw
  } else if ( state == 2 ) {
    dir = 1; // ccw
  } else if ( state == 4 ) {
    dir = 1; // ccw
  } else if (  state == 7 ) {
    dir = -1; // cw
  } else if (  state == 8 ) {
    dir = -1; // cw
  } else if (  state == 11 ) {
    dir = 1; // ccw
  } else if (  state == 13) {
    dir = 1; // ccw
  } else if (  state == 14) {
    dir = -1; // cw
  }
  e1_count += dir;

  // Save current state as old state for next call.
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;

  /*
    //Speed
    e1_interval = micros() - e1_last_time ;
    e1_last_time = micros();

    //1 encoder count equals 0.15mm of wheel travel
    //Personally, i preffer to just to this without units. Recheck later
    if (e1_interval != 0) {
      e1_speed = ((float)1000000) / ((float)e1_interval);
    }
    e1_speed *= dir;
  */
}


//LEFT MOTOR//

// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( PCINT0_vect ) {

  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  boolean newE0_B = PINE & (1 << PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE0_A ^= newE0_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );

  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  int dir = 0; //1 forwards,-1 backwards
  if ( state == 1 ) {
    dir = -1; // cw
  } else if ( state == 2 ) {
    dir = 1; // ccw
  } else if ( state == 4 ) {
    dir = 1; // ccw
  } else if (  state == 7 ) {
    dir = -1; // cw
  } else if (  state == 8 ) {
    dir = -1; // cw
  } else if (  state == 11 ) {
    dir = 1; // ccw
  } else if (  state == 13) {
    dir = 1; // ccw
  } else if (  state == 14) {
    dir = -1; // cw
  }
  e0_count += dir;  // cw

  // Save current state as old state for next call.
  oldE0_A = newE0_A;
  oldE0_B = newE0_B;


  /*

    //Speed
    e0_interval = micros() - e0_last_time ;
    e0_last_time = micros();
    //1 encoder count equals 0.15mm of wheel travel
    //Personally, i preffer to just to this without units. Recheck later
    if (e0_interval != 0) {
    e0_speed = 0.15*1000000L / e0_interval;
    }
    e0_speed *= dir;
  */
}


// TIMER
// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {

  e0_interval = micros() - e0_last_time ;
  e0_last_time = micros();

  //Left motor
  //Change in rotation
  long e0_change = e0_count - prev_e0_count;

  //Multiply by 0.15f to convert to mm/sec
  e0_speed = (float)(e0_change * 1000000L) / (float) e0_interval;
  prev_e0_count = e0_count;

  //Right motor
  long e1_change = e1_count - prev_e1_count;
  e1_speed = (float)(e1_change * 1000000L) / (float) e0_interval; // * timer_freq_hz;
  prev_e1_count = e1_count;

}



//**********************//
//***Setup Interrupts***//
//**********************//

//Setup left encoder
void setupEncoder0() {

  // Initialise our count value to 0.
  e0_count = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldE0_A = 0;
  oldE0_B = 0;

  // Setting up E0_PIN_B:
  // The Romi board uses the pin PE2 (port E, pin 2) which is
  // very unconventional.  It doesn't have a standard
  // arduino alias (like d6, or a5, for example).
  // We set it up here with direct register access
  // Writing a 0 to a DDR sets as input
  // DDRE = Data Direction Register (Port)E
  // We want pin PE2, which means bit 2 (counting from 0)
  // PE Register bits [ 7  6  5  4  3  2  1  0 ]
  // Binary mask      [ 1  1  1  1  1  0  1  1 ]
  //
  // By performing an & here, the 0 sets low, all 1's preserve
  // any previous state.
  DDRE = DDRE & ~(1 << DDE6);
  //DDRE = DDRE & B11111011; // Same as above.

  // We need to enable the pull up resistor for the pin
  // To do this, once a pin is set to input (as above)
  // You write a 1 to the bit in the output register
  PORTE = PORTE | (1 << PORTE2 );
  //PORTE = PORTE | 0B00000100;

  // Encoder0 uses conventional pin 26
  pinMode( E0_A_PIN, INPUT );
  digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor

  // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
  // pin-change interrupts.
  // Note, this register will normally create an interrupt a change to any pins
  // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
  // When we set these registers, the compiler will now look for a routine called
  // ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
  // mismatch to PCINT4, however there is only the one vector servicing a change
  // to all PCINT0->7 pins.
  // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR

  // Page 91, 11.1.5, Pin Change Interrupt Control Register
  // Disable interrupt first
  PCICR = PCICR & ~( 1 << PCIE0 );
  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);

  //Speed
  e0_last_time = micros();
}



//Setup right encoder
void setupEncoder1() {

  // Initialise our count value to 0.
  e0_count = 0;
  e1_count = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldE1_A = 0;
  oldE1_B = 0;

  // Setup pins for encoder 1
  pinMode( E1_A_PIN, INPUT );
  pinMode( E1_B_PIN, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1 << INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.

  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above

}


// Routine to setupt timer3 to run
void setupTimer3() {

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS32);
  //  TCCR3B = TCCR3B | (1 << CS31);
  //  TCCR3B = TCCR3B | (1 << CS30);


  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  //OCR3A = 31250;  //2hz
  OCR3A = 62500 / timer_freq_hz ;
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();

}
