
/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://docs.arduino.cc/hardware/

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/Blink/
*/

//
// ArduinoUniqueID.ino
//
// Example shows the UniqueID on the Serial Monitor.
//

#include <ArduinoUniqueID.h>

#include <errno.h>



// One way to size the stack: just drop some content in there and thus have the .stack segment grow!
// We're using GCC-specific functionality to place this into the .stack section, not where it would normally go.
// The linker script makes sure that the .stack section is at the right place.
__attribute__ ((section(".stack"), used))
static uint8_t StackSpace[9000];


// define the function callback prototype here to twart the ArduinoIDE code preprocessor screwing up the parse order of the final sourcecode.
typedef bool condition_check_t(uint32_t elapsed_ms, uint32_t delay_ms);

void detect_heap_size(bool ok_to_blab = true);
void detect_stack_size(bool ok_to_blab = true);

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



static void dumpUniqueID(bool ok_to_blab)
{
  if (ok_to_blab) {
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

  detect_heap_size();
  detect_stack_size();

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

  dumpUniqueID(time_of_last_user_action == 0);
  
  detect_heap_size(time_of_last_user_action == 0);
  detect_stack_size(time_of_last_user_action == 0);


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

extern "C" {
/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

  extern const void *__malloc_sbrk_base;
  extern const uint32_t __malloc_max_total_mem;
  extern const uint32_t __malloc_max_sbrked_mem;
  extern const void *__malloc_top_pad;
 
  extern const uint32_t _sbss;
  extern const uint32_t _ebss;

  extern const uint32_t _end;

  // extern caddr_t _sbrk ( int incr );

  // while GCC says:    void *_sbrk(ptrdiff_t __incr);
}

// align top address to a 128 bit boundary. Adjust *down* by the given offset, before aligning.
//
// Note: always 'round down'!
static inline const uint8_t *align_addr_to_128bit_boundary(const uint8_t *ptr, int adjustment_in_bytes = 0) {
    intptr_t addr = (intptr_t)ptr;
    addr -= adjustment_in_bytes;
    const intptr_t mask = 0x000F;
    addr &= ~mask;
    return (const uint8_t *)addr;
}

// adjust *down* by given offset, then align. new_top only applies when *above* old_top, i.e. the area bounded by top can only *grow*.
static inline const uint8_t *adjust_top_address(const uint8_t *old_top, const uint8_t * new_top = nullptr, int adjustment_in_bytes = 0) {
    new_top = align_addr_to_128bit_boundary(new_top, adjustment_in_bytes);

    if (!old_top) {
      old_top = new_top;
    }
    else if (old_top < new_top) {
      old_top = new_top;
    }
    return old_top;
}

// adjust *up* by given offset, then align. new_bottom only applies when *below* old_bottom, i.e. the area bounded by bottom can only *grow*.
static inline const uint8_t *adjust_bottom_address(const uint8_t *old_bottom, const uint8_t * new_bottom = nullptr, int adjustment_in_bytes = 0) {
    new_bottom = align_addr_to_128bit_boundary(new_bottom, -adjustment_in_bytes - 15);

    if (!old_bottom) {
      old_bottom = new_bottom;
    }
    else if (old_bottom > new_bottom) {
      old_bottom = new_bottom;
    }
    return old_bottom;
}

static const uint8_t *sbrk_heap_end = NULL;

extern "C" 
caddr_t _sbrk(int incr) {

  if (!debug_print_available) {
    const uint8_t *prev_heap_end;

    if (sbrk_heap_end == NULL) {
        sbrk_heap_end = (const uint8_t *)&_end;
    }
    prev_heap_end = sbrk_heap_end;

    sbrk_heap_end += incr;

    return (caddr_t)prev_heap_end;
  }

  return custom_sbrk(incr);
}

static const uint8_t *find_end_of_last_chunk_of_empty_space_marker_bytes(const uint8_t marker_byte, const uint8_t *heap_end, const uint8_t *current_stack_start, unsigned int safety_margin_below_stack = 64) {

#pragma GCC diagnostic push
// warning: array subscript -16 is outside array bounds of 'unsigned char [4]' [-Warray-bounds=]
#pragma GCC diagnostic ignored "-Warray-bounds"

  current_stack_start -= safety_margin_below_stack;

#pragma GCC diagnostic pop

  if (current_stack_start <= heap_end) {
    return current_stack_start;
  }

  // scan the 'unused memory space' below the current stack for the last occurrence of the marker, *in bulk* (to prevent false positives):
  const uint8_t *p = current_stack_start;
  for (;;) {
    while (p > heap_end && *p != marker_byte) {
      p--;
    }
    // number of consecutive marker bytes which designate a TRUE chunk of yet-unused RAM:
    const int chunk_size = 16;
    if (p < heap_end + chunk_size) {
      return current_stack_start;
    }
    const auto* q = p - 1;
    while (q >= heap_end && *q == marker_byte) {
      q--;
    }
    // now `q` is the first byte below `p` which is NOT a marker byte!
    // meanwhile, `p` points to the very last marker byte of the series...
    if (p - q >= chunk_size) {
      // a true hit! and the top-most one counts, so we're done!
      return p + 1;
    }
    p = q;   // ... and search for 
  }
}

static void *get_current_stack_bottom() {
  // https://stackoverflow.com/questions/57032089/reading-stack-pointer-value-in-arm
  // https://stackoverflow.com/questions/20059673/print-out-value-of-stack-pointer
  // https://gcc.gnu.org/onlinedocs/gcc/Return-Address.html
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=121013
#if __has_builtin(__builtin_stack_address)
  void *p = __builtin_stack_address();
  return p;
#else
  void *p = __builtin_frame_address(0);
  return p;
#endif
}

static void *get_current_stack_bottom_alloca() {
  // https://gcc.gnu.org/onlinedocs/gcc/Stack-Allocation.html
  void *p = __builtin_alloca(4);
#if 0  // GCC v15 warning: function returns address of local variable [-Wreturn-local-addr]; code generator fails at run-time, i.e. this function returns NULL!
  return p;
#else
  (void)p;
  return __builtin_frame_address(0);
#endif
}

static void printZoneAddressHelper(const void *addr) {
  unsigned long v = (unsigned long)addr;
	Serial.print("0x");
	Serial.print(v, 16);
}

static void printZoneAddressMarkerName(const char *name) {
	Serial.print("* ");
	Serial.print(name);
  auto l = strlen(name);
	Serial.print(" ");
	Serial.print("................................................................." + l);
	Serial.print(": ");
}

#define printZoneAddress(marker)                    \
  printZoneAddressMarkerName(#marker);              \
  printZoneAddressHelper((const void *)&marker);    \
	Serial.print("\n")


static constexpr const uint8_t sbrk_marker_byte = 0xC3;


static void report_current_stack_usage() {
  const uint8_t * const stack_end = (const uint8_t *)&_estack;
  unsigned int actual_stack_size = 0;
  const uint8_t * stack_start = (const uint8_t *)&actual_stack_size;
  actual_stack_size = stack_end - stack_start;

  const uint8_t * const max_stack_start = (const uint8_t *)&_sstack;
  int max_stack_size = stack_end - max_stack_start;

  Serial.print("# currently USED stack size = ");
  Serial.print(actual_stack_size);
  Serial.print(" bytes, empty space between heap and stack = ");
  Serial.print(stack_start - sbrk_heap_end);
  Serial.print(" / ");
  Serial.print(max_stack_start - sbrk_heap_end);
  Serial.print(" bytes, max stack size = ");
  Serial.print(max_stack_size);
  Serial.println(" bytes");
}

caddr_t custom_sbrk(int incr) {
  const uint8_t *prev_heap_end;
  const int orig_incr = incr;

  if (incr) {
    Serial.print("SBRK: incr = ");
    Serial.print(incr);
    Serial.println(" bytes");
  }

  const uint8_t * const stack_bottom = (const uint8_t *)&_sstack;

  const uint8_t * const heap_start = (const uint8_t *)&_end;

  static volatile uint8_t init = 0;
  if (!init) {
    init = 2;

#if 0
    sbrk_heap_end = (const uint8_t *)&_end;
#endif

    // We're starting the app: fill the empty/unused space with a special marker byte
    // so we can detect actual stack usage!

    Serial.print("SBRK: fill/mark! _end = ");
    Serial.printf("%p", sbrk_heap_end);

    Serial.print(", current stack bottom = ");
    Serial.printf("%p\n", stack_bottom);

    report_current_stack_usage();

#pragma GCC diagnostic push
// warning: array subscript -2 is outside array bounds of 'unsigned int [4]' [-Warray-bounds=]
#pragma GCC diagnostic ignored "-Warray-bounds"

    auto* fill_top = stack_bottom - 80;   // a tiny safety margin...

#pragma GCC diagnostic pop

    memset((void *)sbrk_heap_end, sbrk_marker_byte, fill_top - sbrk_heap_end);
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

    const uint8_t *current_stack_pos = (const uint8_t *)get_current_stack_bottom();

    report_current_stack_usage();

    // scan the 'empty zone' between heap_end and stack_start for any changes: that's how deep the stack has gone in actuality, i.e. including the active interrupt handlers, etc.!
    const uint8_t *p = find_end_of_last_chunk_of_empty_space_marker_bytes(sbrk_marker_byte, prev_heap_end, current_stack_pos);
    const uint8_t *top = adjust_top_address(sbrk_heap_end, p, 512);

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

    Serial.printf("SBRK: new heap size: %d\n", int(sbrk_heap_end - heap_start));

    find_heap_limit_and_report_search();
  }

  init = init - 1; // 2->1 unless we're a nested call, in which case we go 3->2

  if (incr == 0 && orig_incr > 0) {
#if 0
    __set_errno(ENOMEM);
#else
    errno = ENOMEM;
#endif
    return (caddr_t) -1;
  }

  return (caddr_t)prev_heap_end;
}



void find_heap_limit_and_report_search() {
  const uint8_t * const stack_top = (const uint8_t *)&_estack;
  const uint8_t * const stack_bottom = (const uint8_t *)&_sstack;

  const uint8_t * const heap_start = (const uint8_t *)&_end;

  Serial.printf("SBRK: current stack info:\n");

  const uint8_t *current_stack_pos = (const uint8_t *)get_current_stack_bottom();

  report_current_stack_usage();

  // scan the 'empty zone' between heap_end and stack_start for any changes: that's how deep the stack has gone in actuality, i.e. including the active interrupt handlers, etc.!
  const uint8_t *p = find_end_of_last_chunk_of_empty_space_marker_bytes(sbrk_marker_byte, sbrk_heap_end, current_stack_pos);

  Serial.print("SCAN: marker_byte (0x");
  Serial.print(sbrk_marker_byte, 16);
  Serial.print(") has been changed at index ");
  Serial.print(int(stack_top - p));
  Serial.printf(" --> %d bytes left available on the stack", int(stack_bottom - p));
  Serial.printf(" & %d more bytes have been used vs. right now.\n", int(current_stack_pos - p));
  Serial.print("The marker_byte position is offset ");
  Serial.print(p - sbrk_heap_end);
  Serial.print(" from heap_end.\n");

  Serial.print("heap end = ");
  Serial.printf("%p", sbrk_heap_end);
  Serial.print(", heap start = ");
  Serial.printf("%p", heap_start);

  Serial.print(", stack bottom = ");
  Serial.printf("%p", stack_bottom);
  Serial.print(", current stack pos = ");
  Serial.printf("%p", current_stack_pos);
  Serial.print(", currently USED stack size = ");
  Serial.print(stack_top - current_stack_pos);
  Serial.print(" bytes, LOCATED stack bottom = ");
  Serial.printf("%p", p);
  Serial.print(", ergo: actual used stack size = ");
  Serial.print(stack_top - p);
  Serial.print(" bytes & current empty space between heap and stack = ");
  Serial.print(p - sbrk_heap_end);
  Serial.println(" bytes");

  Serial.printf("heap size: %d\n", int(sbrk_heap_end - heap_start));

  Serial.print("\nZone Addresses:\n");

  printZoneAddress(_sfixed);
  printZoneAddress(_efixed);
  printZoneAddress(_etext);
  printZoneAddress(_srelocate);
  printZoneAddress(_erelocate);
  printZoneAddress(_szero);
  printZoneAddress(_ezero);
  printZoneAddress(_sstack);
  printZoneAddress(_estack);
  printZoneAddress(_sbss);
  printZoneAddress(_ebss);
  printZoneAddress(_end);

  printZoneAddressMarkerName("malloc/free heap end");              
  printZoneAddressHelper((const void *)sbrk_heap_end);    
  Serial.print(" (= ");              
  Serial.print(int(sbrk_heap_end - (const uint8_t *)__malloc_sbrk_base));    
  Serial.print(")\n");

  void *current_sstack = get_current_stack_bottom();
  printZoneAddressMarkerName("current stack end");              
  printZoneAddressHelper(current_sstack);    
  Serial.print(" (= ");              
  Serial.print(-1 * int((const uint8_t *)current_sstack - (const uint8_t *)&_estack));    
  Serial.print(")\n");

  current_sstack = get_current_stack_bottom_alloca();
  printZoneAddressMarkerName("current stack end (v2: alloca based)");              
  printZoneAddressHelper(current_sstack);    
  Serial.print("\n");

  printZoneAddressMarkerName("__malloc_sbrk_base");              
  printZoneAddressHelper((const void *)&__malloc_sbrk_base);    
  Serial.print(" (= ");              
  printZoneAddressHelper(__malloc_sbrk_base);    
  Serial.print(")\n");

  //extern const uint32_t __malloc_max_total_mem;
  printZoneAddressMarkerName("__malloc_max_total_mem");              
  printZoneAddressHelper((const void *)&__malloc_max_total_mem);    
  Serial.print(" (= ");              
  Serial.print(__malloc_max_total_mem);    
  Serial.print(")\n");

  //extern const uint32_t __malloc_max_sbrked_mem;
  printZoneAddressMarkerName("__malloc_max_sbrked_mem");              
  printZoneAddressHelper((const void *)&__malloc_max_sbrked_mem);    
  Serial.print(" (= ");              
  Serial.print(__malloc_max_sbrked_mem);    
  Serial.print(")\n");

  //extern const void *__malloc_top_pad;
  printZoneAddressMarkerName("__malloc_top_pad");              
  printZoneAddressHelper((const void *)&__malloc_top_pad);    
  Serial.print(" (= ");              
  printZoneAddressHelper(__malloc_top_pad);    
  Serial.print(")\n");

  Serial.printf("Calculated heap size: %d\n", int(sbrk_heap_end - (const uint8_t *)&_end));    

#if 0
  // this test code chunk will recursively invoke sbrk() when used inside the sbrk / custom_sbrk functions!
  {
    void *tp = malloc(512);
    Serial.printf("TEST malloc: returned %p\n", tp);
    free(tp);
  }
#endif

  Serial.printf("%60s\n", "----------------------------------------------");

  Serial.print("...(");
  Serial.print(dummyStr.length());
  Serial.print("):...");
  Serial.println(dummyStr.c_str() + max(0, int(dummyStr.length()) - 64));
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

  {
    static int counter = 0;
    if (++counter > 10) {
      // run towards a heap overflow...
      dummyStr += "\n........................................................................................................................................................................................................................................................heap size test #";
      char nstr[20];
      dummyStr += itoa(counter, nstr, 10);
    }
  }
}

void detect_stack_size(bool ok_to_blab) {
  caddr_t base_heap_end = _sbrk(0);

  const uint8_t * const heap_end = (const uint8_t *)base_heap_end;

  const uint8_t * const stack_start = (const uint8_t *)&_sstack;
  const uint8_t * const stack_end = (const uint8_t *)&_estack;

  int stack_size = stack_end - stack_start;

  if (ok_to_blab) {
    Serial.print("application STACK size (= &_estack - &_sstack): ");
    Serial.print(stack_size);
    Serial.print(" bytes\n");

    Serial.print("\nZone Addresses:\n");

    printZoneAddress(_sfixed);
    printZoneAddress(_efixed);
    printZoneAddress(_etext);
    printZoneAddress(_srelocate);
    printZoneAddress(_erelocate);
    printZoneAddress(_szero);
    printZoneAddress(_ezero);
    printZoneAddress(_sstack);
    printZoneAddress(_estack);
    printZoneAddress(_sbss);
    printZoneAddress(_ebss);
    printZoneAddress(_end);

    printZoneAddressMarkerName("malloc/free heap end");              
    printZoneAddressHelper((const void *)heap_end);    
    Serial.print("\n");
  }

  void *current_sstack = get_current_stack_bottom();
  if (ok_to_blab) {
    printZoneAddressMarkerName("current stack end");              
    printZoneAddressHelper(current_sstack);    
    Serial.print("\n");
  }

  current_sstack = get_current_stack_bottom_alloca();
  if (ok_to_blab) {
    printZoneAddressMarkerName("current stack end (v2: alloca based)");              
    printZoneAddressHelper(current_sstack);    
    Serial.print("\n");
  }

  {
    static bool init = false;

    if (!init) {
      unsigned int histogram[256] = {0};
      int count = 0;
      unsigned short int low_idx;
      unsigned int low_val;
      const uint8_t *current_sstack = (const uint8_t *)get_current_stack_bottom_alloca();

      if (ok_to_blab) {
        printZoneAddressMarkerName("current stack end (#3, also alloca based)");              
        printZoneAddressHelper(current_sstack);
        Serial.print("\n");
      }

      for (const uint8_t *p = current_sstack; p < (const uint8_t *)&_estack; p++) {
        histogram[*p]++;
        count++;
      }

      if (ok_to_blab) {
        Serial.print("stack data histogram:\n  stack size/count = ");
        Serial.print(count);
        Serial.println();
      }

      low_idx = 0;
      low_val = histogram[low_idx];
      for (unsigned short int i = 1; i < 256; i++) {
        if (i == 0xFF)
          continue; // do not include byte value 0xFF in the scan: we don't want to see that one score a self-goal as it'll be ubiquitous in the Flash already...

        if (ok_to_blab) {
          Serial.print("[0x");
          Serial.print(i, 16);
          Serial.print("] = ");
          Serial.print(histogram[i]);
        }
        
        if (histogram[i] <= low_val) {
          low_idx = i;
          low_val = histogram[i];

          if (ok_to_blab) {
            Serial.print(" (*)");
          }
        }

        if (ok_to_blab) {
          Serial.println();
        }
      }
      
      // check if our preferred 'magic value' has the lowest score: if it has, use it!
      if (histogram[sbrk_marker_byte] == low_val)
        low_idx = sbrk_marker_byte;

      uint8_t fill_byte = low_idx;

      if (ok_to_blab) {
        Serial.print("-------------------------\nlow @ 0x");
        Serial.print(fill_byte, 16);
        Serial.println();
        Serial.print("-------------------------\nfilling empty RAM: length = ");
        Serial.print(current_sstack - heap_end);
        Serial.println();
      }

      memset((void *)heap_end, sbrk_marker_byte, current_sstack - heap_end);

      init = true;
    }
    else {
      //assert(init);
      const uint8_t *current_sstack = (const uint8_t *)get_current_stack_bottom_alloca();

      if (ok_to_blab) {
        printZoneAddressMarkerName("current stack end (#4, also alloca based)");              
        printZoneAddressHelper(current_sstack);
        Serial.println();
      }

      // scan the 'empty zone' between heap end and stack start for any changes: that's how deep the stack has gone!
      const uint8_t *p;
      const uint8_t *last_edited_p = current_sstack;
      for (p = current_sstack; p > heap_end; p--) {
        if (*p != sbrk_marker_byte) {
          last_edited_p = p;
        }
      }

      if (is_any_user_entry_data_available())
        return;

      if (ok_to_blab) {
        p = last_edited_p;
        Serial.print("SCAN: marker_byte (0x");
        Serial.print(sbrk_marker_byte, 16);
        Serial.print(") has been changed at index ");
        Serial.print(int(current_sstack - p));
        Serial.print(", which is offset ");
        Serial.print(p - heap_end);
        Serial.print(" from heap_end ~ stack offset: ");
        Serial.println(stack_start - p);
      }
    }
  }

  //extern const void *__malloc_sbrk_base;
  if (ok_to_blab) {
    printZoneAddressMarkerName("__malloc_sbrk_base");              
    printZoneAddressHelper((const void *)&__malloc_sbrk_base);    
    Serial.print(" (= ");              
    printZoneAddressHelper(__malloc_sbrk_base);    
    Serial.print(")\n");

    //extern const uint32_t __malloc_max_total_mem;
    printZoneAddressMarkerName("__malloc_max_total_mem");              
    printZoneAddressHelper((const void *)&__malloc_max_total_mem);    
    Serial.print(" (= ");              
    Serial.print(__malloc_max_total_mem);    
    Serial.print(")\n");

    //extern const uint32_t __malloc_max_sbrked_mem;
    printZoneAddressMarkerName("__malloc_max_sbrked_mem");              
    printZoneAddressHelper((const void *)&__malloc_max_sbrked_mem);    
    Serial.print(" (= ");              
    Serial.print(__malloc_max_sbrked_mem);    
    Serial.print(")\n");

    //extern const void *__malloc_top_pad;
    printZoneAddressMarkerName("__malloc_top_pad");              
    printZoneAddressHelper((const void *)&__malloc_top_pad);    
    Serial.print(" (= ");              
    printZoneAddressHelper(__malloc_top_pad);    
    Serial.print(")\n");

    Serial.print("...(");
    Serial.print(dummyStr.length());
    Serial.print("):...");
    Serial.println(dummyStr.c_str() + max(0, int(dummyStr.length()) - 40));
  }
}





/*

                0x0008cad8                . = ALIGN (0x4)
                0x0008cad8                _etext = .

.relocate       0x20070000      0x914 load address 0x0008cad8
                0x20070000                . = ALIGN (0x4)
                0x20070000                _srelocate = .
 *(.ramfunc .ramfunc.*)
 .ramfunc       0x20070000       0x2c C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Reset.cpp.o)
                0x20070000                banzai
 *(.data .data.*)
 .data.SystemCoreClock
                0x2007002c        0x4 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(system_sam3xa.o)
                0x2007002c                SystemCoreClock
 .data._ZL5ticks
                0x20070030        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Reset.cpp.o)
 .data.EndPoints
                0x20070034       0x28 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20070034                EndPoints
 .data._ZL10breakValue
                0x2007005c        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(CDC.cpp.o)
 .data._ZL12_usbLineInfo
                0x20070060        0x8 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(CDC.cpp.o)
 .data.g_interrupt_enabled
                0x20070068        0x4 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(interrupt_sam_nvic.o)
                0x20070068                g_interrupt_enabled
 *fill*         0x2007006c        0x4 
 .data.impure_data
                0x20070070      0x428 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-impure.o)
 .data._impure_ptr
                0x20070498        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-impure.o)
                0x20070498                _impure_ptr
 .data.__malloc_av_
                0x2007049c      0x408 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x2007049c                __malloc_av_
 .data.__malloc_trim_threshold
                0x200708a4        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x200708a4                __malloc_trim_threshold
 .data.__malloc_sbrk_base
                0x200708a8        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x200708a8                __malloc_sbrk_base
 .data.__ctype_ptr__
                0x200708ac        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-ctype_.o)
                0x200708ac                __ctype_ptr__
 .data.lconv    0x200708b0       0x38 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-locale.o)
 .data.lc_ctype_charset
                0x200708e8       0x20 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-locale.o)
 .data.__mb_cur_max
                0x20070908        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-locale.o)
                0x20070908                __mb_cur_max
 .data.__mbtowc
                0x2007090c        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mbtowc_r.o)
                0x2007090c                __mbtowc
 .data.__wctomb
                0x20070910        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-wctomb_r.o)
                0x20070910                __wctomb
                0x20070914                . = ALIGN (0x4)
                0x20070914                _erelocate = .

.bss            0x20070914      0xc24 load address 0x0008d3ec
                0x20070914                . = ALIGN (0x4)
                0x20070914                _sbss = .
                0x20070914                _szero = .
 *(.bss .bss.*)
 .bss           0x20070914       0x1c c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/armv7-m/crtbegin.o
 .bss._UniqueID
                0x20070930       0x10 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\libraries\ArduinoUniqueID\ArduinoUniqueID.cpp.o
                0x20070930                _UniqueID
 .bss.tx_buffer2
                0x20070940       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070940                tx_buffer2
 .bss.tx_buffer1
                0x20070988      0x408 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070988                tx_buffer1
 .bss.Serial    0x20070d90       0x28 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070d90                Serial
 .bss.rx_buffer1
                0x20070db8       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070db8                rx_buffer1
 .bss.Serial1   0x20070e00       0x2c C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070e00                Serial1
 .bss.Serial2   0x20070e2c       0x2c C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070e2c                Serial2
 .bss.Serial3   0x20070e58       0x2c C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070e58                Serial3
 .bss.tx_buffer3
                0x20070e84       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070e84                tx_buffer3
 .bss.tx_buffer4
                0x20070ecc       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070ecc                tx_buffer4
 .bss.g_pinStatus
                0x20070f14       0x4f C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070f14                g_pinStatus
 *fill*         0x20070f63        0x1 
 .bss.rx_buffer2
                0x20070f64       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070f64                rx_buffer2
 .bss.rx_buffer3
                0x20070fac       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070fac                rx_buffer3
 .bss.rx_buffer4
                0x20070ff4       0x48 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF\core\variant.cpp.o
                0x20070ff4                rx_buffer4
 .bss.gpf_isr   0x2007103c        0x4 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(uotghs.o)
                0x2007103c                gpf_isr
 .bss._ZL12port_bitmask
                0x20071040        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Tone.cpp.o)
 .bss._ZL18port_pio_registers
                0x20071044        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Tone.cpp.o)
 .bss._ZL14tone_is_active
                0x20071048        0x1 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Tone.cpp.o)
 *fill*         0x20071049        0x7 
 .bss._ZL11toggleCount
                0x20071050        0x8 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(Tone.cpp.o)
 .bss.callbacksPioB
                0x20071058       0x80 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(WInterrupts.c.o)
 .bss.callbacksPioA
                0x200710d8       0x80 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(WInterrupts.c.o)
 .bss.callbacksPioC
                0x20071158       0x80 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(WInterrupts.c.o)
 .bss.callbacksPioD
                0x200711d8       0x80 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(WInterrupts.c.o)
 .bss.heap.6420
                0x20071258        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(syscalls_sam3.c.o)
 .bss._usbAlternateSetting
                0x2007125c        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x2007125c                _usbAlternateSetting
 .bss._cmark    0x20071260        0x2 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20071260                _cmark
 .bss._ZL14isEndpointHalt
                0x20071262        0x1 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
 *fill*         0x20071263        0x1 
 .bss._cdcComposite
                0x20071264        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20071264                _cdcComposite
 .bss._ZL21isRemoteWakeUpEnabled
                0x20071268        0x1 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
 *fill*         0x20071269        0x3 
 .bss._usbInitialized
                0x2007126c        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x2007126c                _usbInitialized
 .bss.USBDevice
                0x20071270        0x1 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20071270                USBDevice
 *fill*         0x20071271        0x3 
 .bss._usbSetInterface
                0x20071274        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20071274                _usbSetInterface
 .bss._cend     0x20071278        0x2 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x20071278                _cend
 *fill*         0x2007127a        0x2 
 .bss._usbConfiguration
                0x2007127c        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(USBCore.cpp.o)
                0x2007127c                _usbConfiguration
 .bss.cdc_rx_buffer
                0x20071280      0x208 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(CDC.cpp.o)
                0x20071280                cdc_rx_buffer
 .bss._ZZN7Serial_6acceptEvE5guard
                0x20071488        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(CDC.cpp.o)
 .bss.SerialUSB
                0x2007148c       0x14 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(CDC.cpp.o)
                0x2007148c                SerialUSB
 .bss._ZZ12PluggableUSBvE3obj
                0x200714a0        0x8 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(PluggableUSB.cpp.o)
 .bss._ZGVZ12PluggableUSBvE3obj
                0x200714a8        0x4 C:\Users\Ger\AppData\Local\arduino\sketches\76AB1435EA21DFE28FAC64F26A6FABFF/..\..\cores\arduino_sam_arduino_due_x_dbg_7870f019bef9a8f5f8f2c62678d97c7e\core.a(PluggableUSB.cpp.o)
 .bss._dwTickCount
                0x200714ac        0x4 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(timetick.o)
 .bss.ul_recv_fifo_ptr
                0x200714b0       0x28 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(uotghs_device.o)
 .bss.ul_send_fifo_ptr
                0x200714d8       0x28 C:\Users\Ger\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x/libsam_sam3x8e_gcc_rel.a(uotghs_device.o)
 .bss.__malloc_max_total_mem
                0x20071500        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x20071500                __malloc_max_total_mem
 .bss.__malloc_max_sbrked_mem
                0x20071504        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x20071504                __malloc_max_sbrked_mem
 .bss.__malloc_top_pad
                0x20071508        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x20071508                __malloc_top_pad
 .bss.__malloc_current_mallinfo
                0x2007150c       0x28 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-mallocr.o)
                0x2007150c                __malloc_current_mallinfo
 *(COMMON)
 COMMON         0x20071534        0x4 c:/users/ger/appdata/local/arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/../lib/gcc/arm-none-eabi/4.8.3/../../../../arm-none-eabi/lib/armv7-m\libc.a(lib_a-reent.o)
                0x20071534                errno
                0x20071538                . = ALIGN (0x4)
                0x20071538                _ebss = .
                0x20071538                _ezero = .
                0x20071538                . = ALIGN (0x4)
                0x20071538                _end = .

.stack_dummy    0x20071538        0x0
 *(.stack*)
                0x20088000                __StackTop = (ORIGIN (ram) + 0x18000)
                0x20088000                __StackLimit = (__StackTop - SIZEOF (.stack_dummy))
                0x20088000                PROVIDE (_sstack, __StackLimit)
                0x20088000                PROVIDE (_estack, __StackTop)

=====================================================================================================================





port C: pinval.0 =  0
port C: pinstatus = 1
A>
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
malloc/free heap size: 2744 bytes
application STACK size: 0 bytes

Zone Addresses:
* _sfixed .........................: 0x80000
* _efixed .........................: 0x8D23C
* _etext ..........................: 0x8D244
* _srelocate ......................: 0x20070000
* _erelocate ......................: 0x20070914
* _szero ..........................: 0x20070914
* _ezero ..........................: 0x20071548
* _sstack .........................: 0x20088000
* _estack .........................: 0x20088000
* _sbss ...........................: 0x20070914
* _ebss ...........................: 0x20071548
* _end ............................: 0x20071548
* malloc/free heap end ............: 0x20072000
* current stack end ...............: 0x20087F78
* current stack end (v2: alloca based) loc/free heap size: : 0x20087F60
* __malloc_sbrk_base ..............: 0x200708A8*  (=  ............................: 0x20071548)
* __malloc_max_total_mem ..........: 0x20071510*  (=  ............................: 2744)
* __malloc_max_sbrked_mem .........: 0x20071514*  (=  ............................: 2744)
* __malloc_top_pad ................: 0x20071518*  (=  ............................: 0x0)
..........................heap size test #1
A>

...

UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
malloc/free heap size: 2744 bytes
application STACK size: 0 bytes

Zone Addresses:
* _sfixed .........................: 0x80000
* _efixed .........................: 0x8D23C
* _etext ..........................: 0x8D244
* _srelocate ......................: 0x20070000
* _erelocate ......................: 0x20070914
* _szero ..........................: 0x20070914
* _ezero ..........................: 0x20071548
* _sstack .........................: 0x20088000
* _estack .........................: 0x20088000
* _sbss ...........................: 0x20070914
* _ebss ...........................: 0x20071548
* _end ............................: 0x20071548
* malloc/free heap end ............: 0x20072000
* current stack end ...............: 0x20087F78
* current stack end (v2: alloca based) loc/free heap size: : 0x20087F60
* __malloc_sbrk_base ..............: 0x200708A8*  (=  ............................: 0x20071548)
* __malloc_max_total_mem ..........: 0x20071510*  (=  ............................: 2744)
* __malloc_max_sbrked_mem .........: 0x20071514*  (=  ............................: 2744)
* __malloc_top_pad ................: 0x20071518*  (=  ............................: 0x0)
.........................heap size test #10
A>
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
malloc/free heap size: 2744 bytes
application STACK size: 0 bytes

Zone Addresses:
* _sfixed .........................: 0x80000
* _efixed .........................: 0x8D23C
* _etext ..........................: 0x8D244
* _srelocate ......................: 0x20070000
* _erelocate ......................: 0x20070914
* _szero ..........................: 0x20070914
* _ezero ..........................: 0x20071548
* _sstack .........................: 0x20088000
* _estack .........................: 0x20088000
* _sbss ...........................: 0x20070914
* _ebss ...........................: 0x20071548
* _end ............................: 0x20071548
* malloc/free heap end ............: 0x20073000
* current stack end ...............: 0x20087F78
* current stack end (v2: alloca based) loc/free heap size: : 0x20087F60
* __malloc_sbrk_base ..............: 0x200708A8*  (=  ............................: 0x20071548)
* __malloc_max_total_mem ..........: 0x20071510*  (=  ............................: 6840)
* __malloc_max_sbrked_mem .........: 0x20071514*  (=  ............................: 6840)
* __malloc_top_pad ................: 0x20071518*  (=  ............................: 0x0)
.........................heap size test #11
A>
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
UniqueID: 36 04 31 20 34 35 34 55 32 30 39 71 31 35 30 32 
malloc/free heap size: 6840 bytes
application STACK size: 0 bytes

Zone Addresses:
* _sfixed .........................: 0x80000
* _efixed .........................: 0x8D23C
* _etext ..........................: 0x8D244
* _srelocate ......................: 0x20070000
* _erelocate ......................: 0x20070914
* _szero ..........................: 0x20070914
* _ezero ..........................: 0x20071548
* _sstack .........................: 0x20088000
* _estack .........................: 0x20088000
* _sbss ...........................: 0x20070914
* _ebss ...........................: 0x20071548
* _end ............................: 0x20071548
* malloc/free heap end ............: 0x20073000
* current stack end ...............: 0x20087F78
* current stack end (v2: alloca based) loc/free heap size: : 0x20087F60
* __malloc_sbrk_base ..............: 0x200708A8*  (=  ............................: 0x20071548)
* __malloc_max_total_mem ..........: 0x20071510*  (=  ............................: 6840)
* __malloc_max_sbrked_mem .........: 0x20071514*  (=  ............................: 6840)
* __malloc_top_pad ................: 0x20071518*  (=  ............................: 0x0)
.........................heap size test #12

*/