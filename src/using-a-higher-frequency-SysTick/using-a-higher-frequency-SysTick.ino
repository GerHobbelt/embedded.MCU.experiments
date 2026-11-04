
//
// ArduinoUniqueID.ino
//
// Example shows the UniqueID on the Serial Monitor.
//

#include <ArduinoUniqueID.h>
#include <Reset.h>
#include <Arduino.h>

#include <errno.h>

#include "diagnostic_helpers.h"
#include "custom_stack_size.h"



// application override:
extern const uint8_t SystemCoreTickFreqMultiplier = 8; // an application setting which will only be set once, so can be stored in ROM



// define the function callback prototype here to twart the ArduinoIDE code preprocessor screwing up the parse order of the final sourcecode.
typedef bool condition_check_t(uint32_t elapsed_ms, uint32_t delay_ms);




static void establishContact(void) {
  if (Serial.available() <= 0) {
  	Serial.println("A>");  // send a capital A prompt
  }
}


constexpr static const uint32_t pinmask = PIO_PC1 | 0b11111111000011111110;

static void setup_port_C() {
  pmc_enable_periph_clk(ID_PIOC);                                  // To use peripheral, we must enable clock distribution to it

  //PIO_DisableInterrupt(PIOB, PIO_PB27B_TIOB0);
  //PIOB->PIO_IDR |= PIO_PB27B_TIOB0;
  //PIOB->PIO_IER &= ~PIO_PB27B_TIOB0;

  //PIO_PullUp(PIOB, PIO_PB27B_TIOB0, false /* const uint32_t dwPullUpEnable */ );
  //PIOB->PIO_PUER &= ~PIO_PB27B_TIOB0;
  //PIOB->PIO_PUDR |= PIO_PB27B_TIOB0;

  //PIO_SetDebounceFilter(PIOB, PIO_PB27B_TIOB0, 0 /* const uint32_t dwCuttOff */ );

  PIO_SetPeripheral(PIOC, PIO_OUTPUT_0, pinmask);
  PIO_Configure(PIOC, PIO_OUTPUT_0, pinmask, PIO_DEFAULT);
  //PIO_SetInput(PIOC, PIO_PB27B_TIOB0, PIO_DEFAULT);
  PIO_SetOutput(PIOC, pinmask, 0 /* uint32_t dwDefaultValue */, 0 /* uint32_t dwMultiDriveEnable */, 0 /* uint32_t dwPullUpEnable */ );

  // ---------------------------------------------------------------------------------------------------
  // OK, now the same as above, sort of, but by the (datasheet) book: making sure **ALL** PIO registers
  // are set to a Known State, using Figure 31.3 et al, Section 43, with particular focus on Table 31-3
  // in Section 31.7: copy/paste, then rip and clip.
  // Yes, PIO_IMR says 'ReadOnly' but I'm getting tired and anything that's NOT a Status Register gets
  // a write, because the datasheet at least says 'those are harmless' (phrased differently, but anyhow...)

#if 0
  PIOB->PIO_IDR |= PIO_PB27B_TIOB0;   // interrupt disable
  PIOB->PIO_IER &= ~PIO_PB27B_TIOB0;
  PIOB->PIO_IMR &= ~PIO_PB27B_TIOB0;

  PIOB->PIO_AIMDR |= PIO_PB27B_TIOB0;   // interrupt disable
  PIOB->PIO_AIMER &= ~PIO_PB27B_TIOB0;
  PIOB->PIO_AIMMR &= ~PIO_PB27B_TIOB0;

  PIOB->PIO_PDR &= ~PIO_PB27B_TIOB0;  // make sure PIO Controller is in control: clear the 'disable' bit for TIOB0.
  PIOB->PIO_PER |= PIO_PB27B_TIOB0;

  PIOB->PIO_ODR &= ~PIO_PB27B_TIOB0;  // output ENable
  PIOB->PIO_OER |= PIO_PB27B_TIOB0;

  PIOB->PIO_IFER &= ~PIO_PB27B_TIOB0;  
  PIOB->PIO_IFDR |= PIO_PB27B_TIOB0;

  PIOB->PIO_SODR &= ~PIO_PB27B_TIOB0;   // enable output
  PIOB->PIO_CODR |= PIO_PB27B_TIOB0;

  PIOB->PIO_OWDR &= ~PIO_PB27B_TIOB0;   // enable synchronous output
  PIOB->PIO_OWER |= PIO_PB27B_TIOB0;
  // +
  PIOB->PIO_OWSR |= PIO_PB27B_TIOB0;    // see datasheet section 31.5.5 "synchronous output" + 31.5.7
  PIOB->PIO_ODSR &= ~PIO_PB27B_TIOB0;   // enable direct a.k.a. synchronous output

  PIOB->PIO_PUER &= ~PIO_PB27B_TIOB0;  // pullup DISable
  PIOB->PIO_PUDR |= PIO_PB27B_TIOB0;

  //PIO_SetDebounceFilter(PIOB, PIO_PB27B_TIOB0, 0 /* const uint32_t dwCuttOff */ );
  PIOB->PIO_DIFSR &= ~PIO_PB27B_TIOB0; /* set Debouncing, 0 bit field no effect */
  PIOB->PIO_SCIFSR &= ~PIO_PB27B_TIOB0; /* set Deglitching, 0 bit field no effect */

  PIOB->PIO_MDER &= ~PIO_PB27B_TIOB0; // no open collector a.k.a. 'multi drive'...
  PIOB->PIO_MDDR |= PIO_PB27B_TIOB0;
#endif

  volatile auto dummy = PIOC->PIO_ISR;  // dummy read to clear any interrupt glitches during setup
  (void)dummy;

  //auto pinwpstat = PIOC->PIO_WPSR;

  uint32_t pinstatus = PIO_GetOutputDataStatus(PIOC, pinmask);

  PIO_Clear(PIOC, pinmask);
  uint32_t pinval0 = PIO_Get(PIOC, PIO_OUTPUT_0, pinmask);

  //Serial.print("port C: write-protect status register = " << pinwpstat);
  //Serial.println();
  Serial.print("port C: pinval.0 =  ");
  Serial.print(pinval0);
  Serial.println();
  Serial.print("port C: pinstatus = ");
  Serial.print(pinstatus);
  Serial.println();
}



static void dumpUniqueID()
{
  UniqueIDdump(Serial);
  Serial.print("UniqueID: ");
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    if (UniqueID[i] < 0x10)
      Serial.print("0");
    Serial.print(UniqueID[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

//typedef bool condition_check_t(uint32_t elapsed_ms, uint32_t delay_ms);

static bool delay_unless(uint32_t delay_ms, condition_check_t *check)
{
    uint32_t current;
    uint32_t elapsed;
    bool rv = false;

    delay_ms *= SYSTICK_MS_TO_TICKS(1);

    const uint32_t start = GetTickCount();
    do
    {
        current = GetTickCount();
        elapsed = current - start;
        rv = check(elapsed, delay_ms);
    } while (!rv && elapsed < delay_ms);

    return rv;
}

static volatile bool debug_print_available = false;
static volatile uint32_t user_tick_count = 0;
static volatile bool user_tick_pulse = false;

// the setup function runs once when you press reset or power the board
void setup() {
#if 0
  // Set Systick to (1000 / SYSTICK_FREQUENCY) ms interval, common to all SAM3 variants
  if (TimeTick_Configure(SystemCoreClock))
  {
    // Capture error
    while (true);
  }
#endif

	Serial.begin(250000);

  debug_print_available = true;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  setup_port_C();

  detect_stack_size();
  find_heap_limit_and_report_search();

  //establishContact();  // send a byte to establish contact until receiver responds
}

static bool is_any_user_entry_data_available(uint32_t elapsed_ms = 0, uint32_t delay_ms = 0) {
  (void)elapsed_ms;
  (void)delay_ms;

  return (Serial.available() > 0);
}



String dumdum;



// the loop function runs over and over again forever
void loop() {
  static uint32_t time_of_last_user_action = 0;
  // reset the 'interactivity is happening' flag/time to 'inactive' when 8 seconds have passed since the last user activity:
  if (time_of_last_user_action && GetTickCount() > time_of_last_user_action + SYSTICK_MS_TO_TICKS(9000)) {
    time_of_last_user_action = 0;
  }

  if (time_of_last_user_action == 0) {
    dumpUniqueID();
    
    detect_stack_size();
    find_heap_limit_and_report_search();

    Serial.printf("tick counter = %u, SystemCoreClock = %u Hz\n", user_tick_count, SystemCoreClock);
    
    {
      static uint32_t dispcount = 5;

      if (dispcount) {
        for (int k = 0; k < 10; k++) {
          Serial.printf("micros = %u, %u, %u, %u\n", micros(), micros(), micros(), micros());
        }
        Serial.println("###########################################");
        for (int k = 0; k < 10; k++) {
          uint32_t ticks2  = SysTick->VAL;
          bool pend2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || (SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk));
          uint32_t count2  = GetTickCount();
          uint32_t load2 = SysTick->LOAD;

          Serial.printf("ticks = %u, pend = %c, count = %u, load = %u\n", ticks2, (pend2 ? 'Y' : 'n'), count2, load2);
        }

        dispcount--;
      }
    }

    {
      uint32_t load2 = SysTick->LOAD;

      Serial.printf("SysTick LOAD = %u @ %u / %u / %u\n", load2, SystemCoreClock, SYSTICK_FREQUENCY, SystemCoreClock / SYSTICK_FREQUENCY);
    }

    user_tick_count = 0;

    establishContact();  // send a byte to establish contact until receiver responds

    dumdum = "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
             "123456789........................................................................................."
      ;

    time_of_last_user_action = GetTickCount();
  }

  const uint32_t t_now_us = micros();
  const uint32_t t_now = millis();
  if (user_tick_pulse) {
    static uint32_t t_prev_us = micros();
    static uint32_t t_prev = millis();    
    if (t_now - t_prev >= 2000) {
      Serial.printf("delta T = %u ms / %u us\n", t_now - t_prev, t_now_us - t_prev_us);
      print_7seg_display_buffer(); 

      t_prev = t_now;
      t_prev_us = t_now_us;

      user_tick_pulse = false;
    }
  }

  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    time_of_last_user_action = GetTickCount();
    // get incoming byte:
    auto input = Serial.readStringUntil('\n');

    char hexbuf[60];
    snprintf(hexbuf, sizeof(hexbuf), "Recieved: (len:%u) %s", input.length(), input.c_str());
  	Serial.print(strlen(hexbuf));
  	Serial.print("> ");
  	Serial.write(hexbuf);
  	Serial.println();

  	Serial.write(input.c_str());
  	Serial.println();

    decode_7seg_input_str(input.c_str());
  }

  exec_cycle_through_the_7seg_font();

  {
    auto jt = jitter(t_now, 20);

    digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
    delay_unless(40 + jt / 2, is_any_user_entry_data_available);
    jt -= jt / 2;
    digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
    delay_unless(40 + jt, is_any_user_entry_data_available);
  }
}


// produce a semirandom value to use as 'noise/jitter', more specifically as timing jitter so we don't suffer
// from timing aliasing, i.e. show the apparently same 'state' because we alias the interrupt handler's timing
// frequency.
static uint32_t jitter(uint32_t seed, uint32_t range) {
  seed *= 193877777;  // prime: https://en.wikipedia.org/wiki/List_of_prime_numbers
  seed %= range;
  return seed;
}




void detect_heap_size(bool ok_to_blab) {
  caddr_t base_heap_end = _sbrk(0);

  const uint8_t *heap_start = (const uint8_t *)&_end;
  const uint8_t *heap_end = (const uint8_t *)base_heap_end;

  unsigned int heap_size = heap_end - heap_start;

  if (ok_to_blab) {
    Serial.print("malloc/free heap size: ");
    Serial.print(heap_size);
    Serial.print(" bytes\n");
  }
}




/*
    +---+   +---+    +---+  o +---+   
    |   |   |   |  o |   |    |   |   
    +---+   +---+    +---+    +---+   
    |   |   |   |  o |   |    |   |   
    +---+ o +---+ o  +---+ o  +---+ o  

   mapping:

   (hex) for 7 segment digits:

    +-01+   +---+    +---+  o +---+   
   20   02  |   |  o |   |    |   |   
    +-40+   +---+    +---+    +---+   
   10   04  |   |  o |   |    |   |   
    +-08+ o +---+ o  +---+ o  +---+ o  

    numbers and letters (assigned):

                            C
    +-0-+   +---+    +---+  o +---+   
    5   1   |   | Ao |   |    |   |   
    +-6-+   +---+    +---+    +---+   
    4   2   |   | Bo |   |    |   |   
    +-3-+ o +---+ o  +---+ o  +---+ o  
          D       E        F        G

   common cathodes:

    +---+   +---+    +---+  o +---+   
    |   |   |   |  o |   |    |   |   
    +---+   +---+    +---+    +---+   
    |   |   |   |  o |   |    |   |   
    +---+ o +---+ o  +---+ o  +---+ o  
      C1      C2       C3       C4

    +---+   +---+    +---+  o +---+   
    |   |   |   |  o |   |    |   |   
    +---+   +---+    +---+    +---+   
    |   |   |   |  o |   |    |   |   
    +---+ o +---+ o  +---+ o  +---+ o  
                   C5

   port C mapping:
   
   bit 00: ---
   bit 01: C4                 (1 = on, 0 = off)
   bit 02: C5     (the dots)
   bit 03: C3

   bit 04: C2
   bit 05: C1
   bit 06: ---
   bit 07: ---

   bit 08: ---
   bit 09: ---
   bit 10: ---
   bit 11: ---

   bit 12: ---
   bit 13: seg A ~ 0           (1 = off, 0 = on)
   bit 14: seg E ~ 4
   bit 15: seg D ~ 3

   bit 16: seg F ~ 5
   bit 17: seg C ~ 2
   bit 18: seg B ~ 1
   bit 19: seg G ~ 6

   bit 20: ---
   bit 21: ---
   bit 22: ---
   bit 23: ---

   bit 24: ---
   bit 25: ---
   bit 26: ---
   bit 27: ---

   bit 28: ---
   bit 29: ---
   bit 30: ---
   bit 31: ---

*/



enum SegmentSegmentMap : uint8_t {
  SS_NO_SEGMENT = 0,  // nada
  SS_NO_SEGMENT_YET_FLAGGED = 0x01,  // doesn't set any 7 segment LED, but does help to produce a non-zero decode value

  SS_SEG_A = 0x02,  // A
  SS_SEG_B = 0x40,  // B
  SS_SEG_C = 0x20,
  SS_SEG_D = 0x08,
  SS_SEG_E = 0x04,
  SS_SEG_F = 0x10,
  SS_SEG_G = 0x80,   // G

  SS_SEG_C1 = 0x20,
  SS_SEG_C2 = 0x10,
  SS_SEG_C3 = 0x08,
  SS_SEG_C4 = 0x02,
  SS_SEG_C5 = 0x04,

};

static const uint8_t digit_font[] = {
  /* walker anim */
  SS_SEG_A,  // A
  SS_SEG_B,  // B
  SS_SEG_C,
  SS_SEG_D,
  SS_SEG_E,
  SS_SEG_F,
  
  SS_SEG_G,
  
  /* digits */
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F,  // 0
  SS_SEG_B | SS_SEG_C,
  SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_D | SS_SEG_E,
  SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_C | SS_SEG_D,  // 3
  SS_SEG_F | SS_SEG_G | SS_SEG_B | SS_SEG_C,
  SS_SEG_A | SS_SEG_F | SS_SEG_G | SS_SEG_C | SS_SEG_D,
  SS_SEG_A | SS_SEG_G | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F,
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_F,
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F | SS_SEG_G,
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_G | SS_SEG_F,
  
  /* letters, symbols */
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_E | SS_SEG_F, // A
  SS_SEG_G | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F, // b
  SS_SEG_G | SS_SEG_E | SS_SEG_D, // c
  SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_G, // d
  SS_SEG_A | SS_SEG_F | SS_SEG_G | SS_SEG_D | SS_SEG_E, // E
  SS_SEG_A | SS_SEG_G | SS_SEG_E | SS_SEG_F, // F
  SS_SEG_A | SS_SEG_F | SS_SEG_E | SS_SEG_D | SS_SEG_C, // G
  SS_SEG_F | SS_SEG_E | SS_SEG_G | SS_SEG_C, // h
  SS_SEG_C, // i
  SS_SEG_B | SS_SEG_C | SS_SEG_D, // J
  SS_SEG_F | SS_SEG_E | SS_SEG_D, // L
  SS_SEG_G | SS_SEG_C | SS_SEG_E, // n
  SS_SEG_G | SS_SEG_C | SS_SEG_E | SS_SEG_D, // o
  SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_E | SS_SEG_F, // P
  SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_F, // q
  SS_SEG_E | SS_SEG_G, // r
  SS_SEG_G | SS_SEG_D | SS_SEG_E | SS_SEG_F, // t
  SS_SEG_C | SS_SEG_D | SS_SEG_E, // u
  SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_F, // y
};

static const uint8_t dots_font[] = {
  0x20,   // C
  0x02,   // A
  0x40,   // B
  0x08,   // D
  0x0C,   // DE
  0x1C,   // DEF
  0x9C,   // DEFG
};

uint8_t cvt_punct_char_at_P1_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
    return SS_SEG_D;
  }
}

uint8_t cvt_punct_char_at_P2_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
    return SS_SEG_E;
  }
}

uint8_t cvt_punct_char_at_P3_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
    return SS_SEG_B;

  case '`':
  case '\'':
    return SS_SEG_A;

  case ':':
  case ';':
  case '!':
    return SS_SEG_A | SS_SEG_B;
  }
}

uint8_t cvt_punct_char_at_P4_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
    return SS_SEG_F;
  }
}

uint8_t cvt_punct_char_at_P5_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
  case '`':
  case '\'':
    return SS_SEG_C;
  }
}

uint8_t cvt_punct_char_at_P6_to_7seg_bitset(uint8_t c) {
  switch (c) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  case '.':
  case ',':
  case '_':
    return SS_SEG_G;
  }
}


uint8_t cvt_walker_symbol_number_to_7seg_bitset(uint8_t n) {
  switch (n) {
  default:
    // discard/do-not-render unsupported symbols:
    return SS_NO_SEGMENT;

  case 1:
    return SS_SEG_A;

  case 2:
    return SS_SEG_G;
    
  case 3:
    return SS_SEG_D;
    
  case 4:
    return SS_SEG_A | SS_SEG_G;
    
  case 5:
    return SS_SEG_D | SS_SEG_G;
    
  case 6:
    return SS_SEG_A | SS_SEG_G | SS_SEG_D;
    
  case 7:
    return SS_SEG_F;
    
  case 8:
    return SS_SEG_B;
    
  case 9:
    return SS_SEG_E;
    
  case 10:
    return SS_SEG_C;
    
  case 11:
    return SS_SEG_F | SS_SEG_B;
    
  case 12:
    return SS_SEG_E | SS_SEG_C;
    
  case 13:
    return SS_SEG_F | SS_SEG_C;
    
  case 14:
    return SS_SEG_E | SS_SEG_B;
    
  case 15:
    return SS_SEG_F | SS_SEG_E;
    
  case 16:
    return SS_SEG_B | SS_SEG_C;
    
  case 17:
    return SS_SEG_B | SS_SEG_C | SS_SEG_E | SS_SEG_F;
    
  case 20:
    return SS_SEG_A;
    
  case 21:
    return SS_SEG_A | SS_SEG_F;
    
  case 22:
    return SS_SEG_A | SS_SEG_F | SS_SEG_E;
    
  case 23:
    return SS_SEG_A | SS_SEG_F | SS_SEG_E | SS_SEG_D;
    
  case 24:
    return SS_SEG_A | SS_SEG_F | SS_SEG_E | SS_SEG_D | SS_SEG_C;
    
  case 25:
    return SS_SEG_A | SS_SEG_F | SS_SEG_E | SS_SEG_D | SS_SEG_C | SS_SEG_B;
    
  case 26:
    return SS_SEG_A;
    
  case 27:
    return SS_SEG_A | SS_SEG_B;
    
  case 28:
    return SS_SEG_A | SS_SEG_B | SS_SEG_C;
    
  case 29:
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D;
    
  case 30:
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E;
    
  case 31:
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F;
    
  case 32:
    return SS_SEG_A;
    
  case 33:
    return SS_SEG_B;
    
  case 34:
    return SS_SEG_C;
    
  case 35:
    return SS_SEG_D;
    
  case 36:
    return SS_SEG_E;
    
  case 37:
    return SS_SEG_F;
  }
}

// convert input string to 7 segment bitset for a single character
struct char_to_7seg_result_t {
  unsigned segment_bits: 8;
  unsigned consumed: 8;         // number of input characters consumed...
};

uint8_t cvt_basic_alphabet_glyph_to_7seg_bitset(uint8_t c) {
  // toupper:
  if (c >= 'a' && c <= 'z')
    c += 'A' - 'a';

  switch (c) {
  default:
    // do not render unsupported characters:
    return SS_NO_SEGMENT;

  case ' ':
    return SS_NO_SEGMENT_YET_FLAGGED;

  /* digits */
  case '0':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F;  // 0

  case '1':
    return SS_SEG_B | SS_SEG_C;

  case '2':
    return SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_D | SS_SEG_E;

  case '3':
    return SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_C | SS_SEG_D;  // 3

  case '4':
    return SS_SEG_F | SS_SEG_G | SS_SEG_B | SS_SEG_C;

  case '5':
    return SS_SEG_A | SS_SEG_F | SS_SEG_G | SS_SEG_C | SS_SEG_D;

  case '6':
    return SS_SEG_A | SS_SEG_G | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F;

  case '7':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_F;

  case '8':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F | SS_SEG_G;

  case '9':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_G | SS_SEG_F;

  /* letters, symbols */

  case 'A':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_E | SS_SEG_F; // A

  case 'B':
    return SS_SEG_G | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_F; // b

  case 'C':
    return SS_SEG_G | SS_SEG_E | SS_SEG_D; // c

  case 'D':
    return SS_SEG_B | SS_SEG_C | SS_SEG_D | SS_SEG_E | SS_SEG_G; // d

  case 'E':
    return SS_SEG_A | SS_SEG_F | SS_SEG_G | SS_SEG_D | SS_SEG_E; // E

  case 'F':
    return SS_SEG_A | SS_SEG_G | SS_SEG_E | SS_SEG_F; // F

  case 'G':
    return SS_SEG_A | SS_SEG_F | SS_SEG_E | SS_SEG_D | SS_SEG_C; // G

  case 'H':
    return SS_SEG_F | SS_SEG_E | SS_SEG_G | SS_SEG_C; // h

  case 'I':
    return SS_SEG_C; // i

  case 'J':
    return SS_SEG_B | SS_SEG_C | SS_SEG_D; // J

  // K

  case 'L':
    return SS_SEG_F | SS_SEG_E | SS_SEG_D; // L

  // M

  case 'N':
    return SS_SEG_G | SS_SEG_C | SS_SEG_E; // n

  case 'O':
    return SS_SEG_G | SS_SEG_C | SS_SEG_E | SS_SEG_D; // o

  case 'P':
    return SS_SEG_A | SS_SEG_B | SS_SEG_G | SS_SEG_E | SS_SEG_F; // P

  case 'Q':
    return SS_SEG_A | SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_F; // q

  case 'R':
    return SS_SEG_E | SS_SEG_G; // r

  // S

  case 'T':
    return SS_SEG_G | SS_SEG_D | SS_SEG_E | SS_SEG_F; // t

  case 'U':
    return SS_SEG_C | SS_SEG_D | SS_SEG_E; // u

  // V
  // W
  // X

  case 'Y':
    return SS_SEG_B | SS_SEG_C | SS_SEG_G | SS_SEG_F; // y

  // Z

  case '-':
    return SS_SEG_G;

  case '_':
    return SS_SEG_D;
  }
}


struct char_to_7seg_result_t cvt_glyph_to_7_segment_bits(const char *str) {
  uint8_t c = *str++;
  if (c == 0) {
    return { 
      .segment_bits = 0,
      .consumed = 0
    };
  }
  // toupper:
  if (c >= 'a' && c <= 'z')
    c += 'A' - 'a';

  if (c != 'W') {
    auto d = cvt_basic_alphabet_glyph_to_7seg_bitset(c);
    return {
      .segment_bits = d,
      .consumed = (d != 0)
    };
  }
  else {
    // 'walker symbols':
    c = *str++;
    if (c >= '0' && c <= '9') {
      uint8_t d = *str++;
      if (d >= '0' && d <= '9') {
        // double-digit walker symbol number
        d -= '0';
        c -= '0';
        d += c * 10;

        return {
          cvt_walker_symbol_number_to_7seg_bitset(d),
          3
        };
      }
      else {
        // single-digit walker symbol number:
        c -= '0';
        return {
          cvt_walker_symbol_number_to_7seg_bitset(c),
          2
        };
      }
    }
    else {
      // not a walker symbol: discard!
      return { 
        .segment_bits = 0,
        .consumed = 1
      };
    }
  }
}

static bool is_whitespace(uint8_t c) {
  return (c == ' ' || c == '\t');
}

// '1.2.:3.4' -- format [0-9].[0-9].[:.,`'][0-9].['`][0-9].
// where [0-9] is the alphabet 0..9AbcdEFGhiLnoPqrtuy_- plus the 'walker set' w0..w35 
//
// Note: use a ' ' space char to signal you don't want to see anything at that position.

struct display_7seg_bitmap_t {
  uint8_t segment_bits[5]   = {0};
};

struct str_to_7seg_result_t {
  display_7seg_bitmap_t display_buffer;
  uint8_t consumed          = 0;           // number of input characters consumed...
};

struct str_to_7seg_result_t cvt_string_to_7_segment_display(const char *str) {
  str_to_7seg_result_t rv;

  const char *p = str;

  // decode to 7 segment display display bit set: start with the first char/glyph: [0-9]
  struct char_to_7seg_result_t cb = cvt_glyph_to_7_segment_bits(p);
  rv.display_buffer.segment_bits[0] = cb.segment_bits;
  p += cb.consumed;

  // decode the first dot (optional):
  uint8_t c = *p++;
  uint8_t pc;
  if (c == 0) 
    goto failed;
  pc = cvt_punct_char_at_P1_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] = pc;
  }
  else {
    p--;
  }

  cb = cvt_glyph_to_7_segment_bits(p);
  rv.display_buffer.segment_bits[1] = cb.segment_bits;
  p += cb.consumed;

  // decode the second dot (optional):
  c = *p++;
  if (c == 0) 
    goto failed;
  pc = cvt_punct_char_at_P2_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] |= pc;
  }
  else {
    p--;
  }

  // ... and decode the colon (optional):
  c = *p++;
  if (c == 0) 
    goto failed;
  pc = cvt_punct_char_at_P3_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] |= pc;
  }
  else {
    p--;
  }

  cb = cvt_glyph_to_7_segment_bits(p);
  rv.display_buffer.segment_bits[2] = cb.segment_bits;
  p += cb.consumed;

  // decode the third dot (optional):
  c = *p++;
  if (c == 0) 
    goto failed;
  pc = cvt_punct_char_at_P4_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] |= pc;
  }
  else {
    p--;
  }

  // ... and decode the top dot:
  c = *p++;
  if (c == 0) 
    goto failed;
  pc = cvt_punct_char_at_P5_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] |= pc;
  }
  else {
    p--;
  }

  cb = cvt_glyph_to_7_segment_bits(p);
  rv.display_buffer.segment_bits[3] = cb.segment_bits;
  p += cb.consumed;

  // decode the last dot (optional):
  c = *p++;
  pc = cvt_punct_char_at_P6_to_7seg_bitset(c);
  if (pc) {
    rv.display_buffer.segment_bits[4] |= pc;
  }
  else {
    p--;
  }

failed:
  rv.consumed = p - str;
  return rv;
}





static volatile struct display_7seg_bitmap_t displaybuf_for_7seg_diagblock;
static volatile uint8_t displaybuf_for_7seg_diagblock_state = 0;
static volatile uint32_t displaybuf_for_7seg_diagblock_update_speed = SYSTICK_MS_TO_TICKS(500);



static int cycle_through_the_7seg_font = 0;
static uint32_t t_cycle_last_update = 0;

void print_7seg_display_buffer(void) {
  Serial.printf("7SEG display buffer: [%02x, %02x, %02x, %02x, %02x], state = %u\n", (unsigned int)displaybuf_for_7seg_diagblock.segment_bits[0], (unsigned int)displaybuf_for_7seg_diagblock.segment_bits[1], (unsigned int)displaybuf_for_7seg_diagblock.segment_bits[2], (unsigned int)displaybuf_for_7seg_diagblock.segment_bits[3], (unsigned int)displaybuf_for_7seg_diagblock.segment_bits[4], (unsigned int)displaybuf_for_7seg_diagblock_state);
}

void exec_cycle_through_the_7seg_font() {
  if (cycle_through_the_7seg_font == 0)
    return;

  uint32_t t = GetTickCount();
  if (t < t_cycle_last_update)
    return;
  t_cycle_last_update = t + SYSTICK_MS_TO_TICKS(330);

  char cmdbuf[30];
  static const char chmap[] = "1234567890abcdefghijklmnopqrstuvwxyz_-()[]=|";

  int pos = cycle_through_the_7seg_font - 1;
  cycle_through_the_7seg_font++;
  if (pos < int(sizeof(chmap))) {
    snprintf(cmdbuf, sizeof(cmdbuf), "%c.%c.%c.%c", chmap[pos], chmap[pos], chmap[pos], chmap[pos]);
  }
  else {
    pos -= sizeof(chmap);
    snprintf(cmdbuf, sizeof(cmdbuf), "%02d:w%d w%d", pos, pos, pos);
    if (pos > 48) {
      cycle_through_the_7seg_font = 1; // wrap around...
    }
  }

  auto d = cvt_string_to_7_segment_display(cmdbuf);

  Serial.printf("Decoded: [%02x, %02x, %02x, %02x, %02x], %u, input: '%s', leftover: '%s'\n", (unsigned int)d.display_buffer.segment_bits[0], (unsigned int)d.display_buffer.segment_bits[1], (unsigned int)d.display_buffer.segment_bits[2], (unsigned int)d.display_buffer.segment_bits[3], (unsigned int)d.display_buffer.segment_bits[4], (unsigned int)d.consumed, cmdbuf, cmdbuf + d.consumed);

  // display each 7seg C1..C5 in a sequence in an interrupt routine
  memcpy((void *)displaybuf_for_7seg_diagblock.segment_bits, d.display_buffer.segment_bits, sizeof(displaybuf_for_7seg_diagblock.segment_bits));
}


void decode_7seg_input_str(const char *str) {
  // strip off leading whitespace
  while (is_whitespace(*str))
    str++;

  if (*str == '+') {
    int speed = atoi(str);
    if (speed < 1)
      speed = 1;
    else if (speed > SYSTICK_MS_TO_TICKS(4000))
      speed = SYSTICK_MS_TO_TICKS(4000);

    displaybuf_for_7seg_diagblock_update_speed = speed;
  }
  else if (*str == 'z') {
    // special commands...
    switch (str[1]) {
    case 'c':
      cycle_through_the_7seg_font = 1;
      t_cycle_last_update = GetTickCount() + SYSTICK_MS_TO_TICKS(330);
      break;
    }
  }
  else {
    cycle_through_the_7seg_font = 0;
    t_cycle_last_update = 0;

    auto d = cvt_string_to_7_segment_display(str);

    Serial.printf("Decoded: [%02x, %02x, %02x, %02x, %02x], %u, input: '%s', leftover: '%s'\n", (unsigned int)d.display_buffer.segment_bits[0], (unsigned int)d.display_buffer.segment_bits[1], (unsigned int)d.display_buffer.segment_bits[2], (unsigned int)d.display_buffer.segment_bits[3], (unsigned int)d.display_buffer.segment_bits[4], (unsigned int)d.consumed, str, str + d.consumed);

    // display each 7seg C1..C5 in a sequence in an interrupt routine
    memcpy((void *)displaybuf_for_7seg_diagblock.segment_bits, d.display_buffer.segment_bits, sizeof(displaybuf_for_7seg_diagblock.segment_bits));
  }
}


int sysTickHook(void)
{
  SysTick_Handler_injected();

  return false;
}


// override weak system dummy to hook into the system tick interrupt
void SysTick_Handler_injected(void)
{
  user_tick_count = user_tick_count + 1;

  if (user_tick_count % displaybuf_for_7seg_diagblock_update_speed == 0) {
    user_tick_pulse = true;

    uint8_t i = displaybuf_for_7seg_diagblock_state + 1;
#if 0
    if (i >= 5)
      i = 0;
    displaybuf_for_7seg_diagblock_state = i;
#else
    // test visuals on the 7seg display while mimicking the grid scan on a LinnStrument, so we can test/view tremors and other visual 'dirt':

    if (i >= 26)  // LinnStrument has a 26x8 LED matrix and we (ab)use this codebase to visually test our lighting idea/concept now...
      i = 0;
    displaybuf_for_7seg_diagblock_state = i;
    if (i >= 5) {
      PIO_Clear(PIOC, pinmask);
      return;
    }
#endif

    static const uint8_t cc_line[5] = {
      SS_SEG_C1,
      SS_SEG_C2,
      SS_SEG_C3,
      SS_SEG_C4,
      SS_SEG_C5,
    };
    uint32_t v = 0xFF ^ displaybuf_for_7seg_diagblock.segment_bits[i];
    v <<= 12;
    v |= cc_line[i];

    PIO_Clear(PIOC, pinmask);
    PIO_Set(PIOC, pinmask & v);
  }
}
