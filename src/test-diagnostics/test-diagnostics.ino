
//
// ArduinoUniqueID.ino
//
// Example shows the UniqueID on the Serial Monitor.
//

#include <ArduinoUniqueID.h>

#include <errno.h>

#include "diagnostic_helpers.h"


#if 0
// One way to size the stack: just drop some content in there and thus have the .stack segment grow!
// We're using GCC-specific functionality to place this into the .stack section, not where it would normally go.
// The linker script makes sure that the .stack section is at the right place.
__attribute__ ((section(".stack"), used))
static uint8_t StackSpace[9000 & ~0x00FF];
#endif


// define the function callback prototype here to twart the ArduinoIDE code preprocessor screwing up the parse order of the final sourcecode.
typedef bool condition_check_t(uint32_t elapsed_ms, uint32_t delay_ms);


/*
  Serial Call and Response
  Language: Wiring/Arduino

  This program sends an ASCII A (byte of value 65) on startup and repeats that
  until it gets some data in. Then it waits for a byte in the serial port, and
  sends three sensor values whenever it gets a byte in.

  The circuit:
  - potentiometers attached to analog inputs 0 and 1
  - pushbutton attached to digital I/O 2

  created 26 Sep 2005
  by Tom Igoe
  modified 24 Apr 2012
  by Tom Igoe and Scott Fitzgerald
  Thanks to Greg Shakar and Scott Fitzgerald for the improvements

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/communication/SerialCallResponse/
*/


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

// the setup function runs once when you press reset or power the board
void setup() {
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

  establishContact();  // send a byte to establish contact until receiver responds
}

static bool is_any_user_entry_data_available(uint32_t elapsed_ms = 0, uint32_t delay_ms = 0) {
  (void)elapsed_ms;
  (void)delay_ms;

  return (Serial.available() > 0);
}



// the string that will grow ad infinitum, or rather: until the heap allocation BREAKS. 
// We use this to test the heap allocation logic and slowly run the (growing) heap into the stack,
// resulting in an application CRASH!
String dummyStr;



// the loop function runs over and over again forever
void loop() {
  static uint32_t time_of_last_user_action = 0;
  // reset the 'interactivity is happening' flag/time to 'inactive' when 8 seconds have passed since the last user activity:
  if (time_of_last_user_action && GetTickCount() > time_of_last_user_action + 8000) {
    time_of_last_user_action = 0;
  }

  if (time_of_last_user_action == 0) {
    dumpUniqueID();
    
    detect_stack_size();
    find_heap_limit_and_report_search();

    slowly_consume_heap_space();
  }


  {
    void *tp = malloc(512);
    Serial.printf("LOOP:TEST malloc: returned %p\n", tp);
    if (!tp)
      delay(5000);
    free(tp);
  }
  

  establishContact();  // send a byte to establish contact until receiver responds
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    time_of_last_user_action = GetTickCount();
    // get incoming byte:
    auto input = Serial.readString();

    char hexbuf[60];
    snprintf(hexbuf, sizeof(hexbuf), "Recieved: (len:%u) %s", input.length(), input.c_str());
  	Serial.print(strlen(hexbuf));
  	Serial.print("> ");
  	Serial.write(hexbuf);
  	Serial.println();

  	Serial.write(input.c_str());
  	Serial.println();

    dummyStr += input;      // run towards a heap overflow...

    unsigned int v;
    sscanf(input.c_str(), "%X", &v);
    snprintf(hexbuf, sizeof(hexbuf), "Decoded: 0x%04X", v);
  	Serial.print(strlen(hexbuf));
  	Serial.print("> ");
  	Serial.write(hexbuf);
  	Serial.println();

    dummyStr += hexbuf;      // run towards a heap overflow...

    // unless the input is the CLR command:
    if (input == "CLR\n") {
      dummyStr.reset();
      dummyStr = "";
    	Serial.println("############# CLEAR DUUMY STRING ###############");
    }

    PIO_Clear(PIOC, pinmask);
    PIO_Set(PIOC, pinmask & v);
  }

  digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  delay_unless(50, is_any_user_entry_data_available);
  digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  delay_unless(50, is_any_user_entry_data_available);
}


static const uint8_t seven_segment_map[] = {
  0,  // nada
  0x02,  // A
  0x40,  // B
  0x20,
  0x08,
  0x04,
  0x10,
  0x80   // G
};

static const uint8_t digit_font[] = {
  /* walker anim */
  0x01,
  0x02,
  0x04,
  0x08,
  0x10,
  0x20,
  
  0x03,
  0x06,
  0x0C,
  0x18,
  0x30,
  0x21,
  
  0x23,
  0x07,
  0x0E,
  0x1C,
  0x38,
  0x31,
  
  0,
  0x40,
  0,
  
  /* digits */
  0x3F,  // 0
  0x06,
  0x5B,
  0x4F,  // 3
  0x66,
  0x6D,
  0x7D,
  0x27,
  0x7F,
  0x6F,
  
  /* letters, symbols */
  0,
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

const uint8_t *sbrk_heap_end = NULL;

extern "C" 
caddr_t _sbrk(int incr) {
  static bool inside_sbrk_call = false;

  if (inside_sbrk_call || !debug_print_available) {
    const uint8_t *prev_heap_end;

    if (sbrk_heap_end == NULL) {
        sbrk_heap_end = (const uint8_t *)&_end;
    }
    prev_heap_end = sbrk_heap_end;

    sbrk_heap_end += incr;

    // protection
    const uint8_t * const stack_start = (const uint8_t *)&_sstack;
    if (incr != 0 && sbrk_heap_end >= stack_start) {
#if 0
      __set_errno(ENOMEM);
#else
      errno = ENOMEM;
#endif
      return (caddr_t) -1;
    }

    return (caddr_t)prev_heap_end;
  }

  inside_sbrk_call = true;
  caddr_t rv = custom_sbrk(incr);
  inside_sbrk_call = false;
  return rv;
}

caddr_t custom_sbrk(int incr) {
  const uint8_t *prev_heap_end;
  const int orig_incr = incr;

  report_current_stack_usage();

  if (incr) {
    Serial.print("\n\nSBRK *start* : incr = ");
    Serial.print(incr);
    Serial.println(" bytes\n");
  }

  const uint8_t * const heap_start = (const uint8_t *)&_end;

  static volatile uint8_t init = 0;
  if (!init) {
    init = 2;
  }
  else {
    //assert(init);
    init = init + 1;  // 1->2 unless we're a nested call due to (hidden?) malloc in the debug code below...

    if (init > 2) {
      Serial.print("SBRK: ######## NESTED CALL: REJECT! #########\n");
      incr = 0;
    }
  }

  prev_heap_end = sbrk_heap_end;

  if (incr != 0) {
    find_heap_limit_and_report_search();

    sbrk_heap_end += incr;

    // protect/limit growth: make sure we don't run the heap into the stack,
    // while we also check/adjust the top by using the lowest known-used stack address:

    Serial.printf("SBRK: current stack info:\n");

    const uint8_t *current_stack_pos = (const uint8_t *)get_current_stack_bottom_alloca();

    // scan the 'empty zone' between heap_end and stack_bottom for any changes: that's how deep the stack has gone in actuality, i.e. including the active interrupt handlers, etc.!
    const uint8_t *p = find_end_of_chunk_of_empty_space_marker_bytes(get_sbrk_marker_byte_ref(), sbrk_heap_end, current_stack_pos);
    const uint8_t *top = adjust_address_downwards(sbrk_heap_end, p, 512);

    const uint8_t * const stack_bottom = (const uint8_t *)&_sstack;
    Serial.printf("\nADJUST DOWN: old: %p, new: %p, located: %p", sbrk_heap_end, prev_heap_end, p);
    Serial.printf(" -> %p", top);
    Serial.printf(" ~~: %d / %d / %d // %d\n", int(p - prev_heap_end), int(top - prev_heap_end), int(p - top), int(stack_bottom - top));

#if 01
    // if the preconfigured/built-in stack size is larger still, we should make sure the heap NEVER occupies the same space:
    if (top > stack_bottom) {
      Serial.println("+ ENSURE heap end remains below the stack!");
      top = stack_bottom;
    }
#endif

    // don't merge these into a single Serial.printf() call, as than we'll overflow internal buffers and the output will be clipped:
    Serial.printf("SBRK: incr = %d", incr);
    Serial.printf(", RAW calculated heap end = %p, previous heap end = %p", sbrk_heap_end, prev_heap_end);
    Serial.printf(", top = %p\n", top);
    sbrk_heap_end = top;
    Serial.printf("SBRK: actual incr (after top clipping) = %d\n", int(sbrk_heap_end - prev_heap_end));

    incr = int(sbrk_heap_end - prev_heap_end);
    // we cannot REDUCE the heap, for that's asking for spurious trouble!
    if (incr < 0) {
      Serial.printf("SBRK: !!!! NEGATIVE incr !!! incr = %d", incr);
      incr = 0;
      sbrk_heap_end = prev_heap_end;
    }
    if (incr != orig_incr) {
      Serial.printf("SBRK: !!!! FAIL !!! incr has changed: %d <-> %d\n", incr, orig_incr);

      incr = 0;
      sbrk_heap_end = prev_heap_end;
    }

    Serial.printf("SBRK: new heap size: %d\n", int(sbrk_heap_end - heap_start));

    find_heap_limit_and_report_search();
  }

  init = init - 1; // 2->1 unless we're a nested call, in which case we go 3->2

  if (orig_incr != 0) {
    Serial.print("\n\nSBRK *end* : incr = ");
    Serial.print(incr);
    if (incr == 0) {
      Serial.print(" (FAIL!)");
    }
    Serial.println(" bytes\n");
  }

  //if (incr == 0 && orig_incr > 0) {
  if (incr != orig_incr) {
#if 0
    __set_errno(ENOMEM);
#else
    errno = ENOMEM;
#endif
    return (caddr_t) -1;
  }

  return (caddr_t)prev_heap_end;
}


void slowly_consume_heap_space() {
  Serial.printf("TEST malloc: %s\n", __func__);

  static int counter = 0;
  if (++counter > 5) {
    // run towards a heap overflow...
    dummyStr += "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "\n........................................................................................................................................................................................................................................................"
                "heap size test #";
    char nstr[20];
    dummyStr += itoa(counter, nstr, 10);
  }
}




