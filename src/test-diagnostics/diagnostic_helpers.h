
#pragma once

#include <ArduinoUniqueID.h>


// the string that will grow ad infinitum, or rather: until the heap allocation BREAKS. 
// We use this to test the heap allocation logic and slowly run the (growing) heap into the stack,
// resulting in an application CRASH!
extern String dummyStr;


extern "C" {
  /* Initialize segments */
  extern const void *__malloc_sbrk_base;
  extern const uint32_t __malloc_max_total_mem;
  extern const uint32_t __malloc_max_sbrked_mem;
  extern const void *__malloc_top_pad;

  // extern caddr_t _sbrk ( int incr );

  // while GCC says:    void *_sbrk(ptrdiff_t __incr);
}

// align top address to a 128 bit boundary. Adjust *down* by the given offset, before aligning.
//
// Note: always 'round down'!
const uint8_t *align_addr_to_128bit_boundary(const uint8_t *ptr, int adjustment_in_bytes = 0) {
    intptr_t addr = (intptr_t)ptr;
    addr -= adjustment_in_bytes;
    const intptr_t mask = 0x000F;
    addr &= ~mask;
    return (const uint8_t *)addr;
}

// adjust *down* by given offset, then align. new_top only applies when *above* old_top, i.e. the area bounded by top can only *grow*.
const uint8_t *adjust_address_upwards(const uint8_t *old_top, const uint8_t * new_top = nullptr, int adjustment_in_bytes = 0) {
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
const uint8_t *adjust_address_downwards(const uint8_t *old_bottom, const uint8_t * new_bottom = nullptr, int adjustment_in_bytes = 0) {
    new_bottom = align_addr_to_128bit_boundary(new_bottom, -adjustment_in_bytes - 15);

    if (!old_bottom) {
      old_bottom = new_bottom;
    }
    else if (old_bottom > new_bottom) {
      old_bottom = new_bottom;
    }
    return old_bottom;
}



// Notes:
// Because userland code MAY allocate stack space in any function BUT *not* clobber it, our previous 'scan from the top downwards to find a series of consecutive marker bytes'
// approach was fatally flawed: we MUST scan from the bottom (heap end) upwards and stop once we run out of marker bytes...
//

#if 0

const uint8_t *find_end_of_last_chunk_of_empty_space_marker_bytes(const uint8_t marker_byte, const uint8_t *heap_end, const uint8_t *current_stack_start, unsigned int safety_margin_below_stack = 64) {

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

#endif

const uint8_t *find_end_of_chunk_of_empty_space_marker_bytes(const uint8_t marker_byte, const uint8_t *heap_end, const uint8_t *current_stack_bottom) {
  if (current_stack_bottom <= heap_end) {
    return heap_end;
  }

  // scan the 'unused memory space' below the current stack for the last occurrence of the marker, *in bulk* (to prevent false positives):
  const uint8_t *p;
  for (p = heap_end; p < current_stack_bottom && *p == marker_byte; p++)
      ;
  return p;
}

void *get_current_stack_bottom() {
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

void *get_current_stack_bottom_alloca() {
  // https://gcc.gnu.org/onlinedocs/gcc/Stack-Allocation.html
  void *p = __builtin_alloca(4);
#if 0  // GCC v15 warning: function returns address of local variable [-Wreturn-local-addr]; code generator fails at run-time, i.e. this function returns NULL!
  return p;
#else
  (void)p;
  return __builtin_frame_address(0);
#endif
}

void printZoneAddressHelper(const void *addr) {
  unsigned long v = (unsigned long)addr;
  Serial.print("0x");
  Serial.print(v, 16);
}

void printZoneAddressMarkerName(const char *name) {
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


uint8_t &get_sbrk_marker_byte_ref() {
  static uint8_t marker_byte = 0;  // init flag (0) and fill_byte used to fill/mark the unused space (non-zero, not 0xFF)
  return marker_byte;
}


void report_current_stack_usage() {
  const uint8_t * const stack_top = (const uint8_t *)&_estack;
  const uint8_t * const stack_bottom = (const uint8_t *)&_sstack;
  unsigned int actual_stack_size = 0;
  const uint8_t * stack_pos = (const uint8_t *)&actual_stack_size;
  actual_stack_size = stack_top - stack_pos;

  int max_stack_size = stack_top - stack_bottom;

  // Note: of the three reported stack sizes below (*used* stack, not declared stack space), the *third* is the worst,
  // hence the fill/mark logic will use that API to get a pointer to the bottom of the currently used stack.
  
  Serial.print("# currently USED stack size = ");
  Serial.print(actual_stack_size);

  const uint8_t *current_stack_pos = (const uint8_t *)get_current_stack_bottom();
  Serial.print(" / ");
  Serial.print(stack_top - current_stack_pos);

  current_stack_pos = (const uint8_t *)get_current_stack_bottom_alloca();
  Serial.print(" / ");
  Serial.print(stack_top - current_stack_pos);

  caddr_t base_heap_end = _sbrk(0);

  //const uint8_t *heap_start = (const uint8_t *)&_end;
  const uint8_t *heap_end = (const uint8_t *)base_heap_end;

  Serial.print(" bytes, empty space between heap and stack = ");
  Serial.print(stack_pos - heap_end);
  Serial.print(" / ");
  Serial.print(stack_bottom - heap_end);

  if (int(stack_bottom - heap_end) < 0)
    Serial.print(" (FAIL!)");

  Serial.print(" bytes, max stack size = ");
  Serial.print(max_stack_size);
  Serial.println(" bytes");
}

void find_heap_limit_and_report_search() {
  const uint8_t * const stack_top = (const uint8_t *)&_estack;
  const uint8_t * const stack_bottom = (const uint8_t *)&_sstack;

  caddr_t base_heap_end = _sbrk(0);

  const uint8_t *heap_start = (const uint8_t *)&_end;
  const uint8_t *heap_end = (const uint8_t *)base_heap_end;

  Serial.printf("Current stack info:\n");

  const uint8_t *current_stack_pos = (const uint8_t *)get_current_stack_bottom_alloca();

  report_current_stack_usage();

  // scan the 'empty zone' between heap_end and stack_bottom for any changes: that's how deep the stack has gone in actuality, i.e. including the active interrupt handlers, etc.!
  const uint8_t *p = find_end_of_chunk_of_empty_space_marker_bytes(get_sbrk_marker_byte_ref(), heap_end, current_stack_pos);

  Serial.print("SCAN: marker_byte (0x");
  Serial.print(get_sbrk_marker_byte_ref(), 16);
  Serial.print(") has been changed at index ");
  Serial.print(int(stack_top - p));
  Serial.printf(" --> %d bytes left available on the stack", int(p - stack_bottom));
  Serial.printf(" & %d more bytes have been used vs. right now.\n", int(current_stack_pos - p));
  Serial.print("The marker_byte position is offset ");
  Serial.print(p - heap_end);

  if (int(p - heap_end) <= 0)
    Serial.print(" (DAMAGE/FAIL?!)");

  Serial.print(" from heap_end.\n");

  Serial.print("heap end = ");
  Serial.printf("%p", heap_end);
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
  Serial.print("\nErgo: actual used stack size = ");
  Serial.print(stack_top - p);
  Serial.print(" bytes & current empty space between heap and stack = ");
  Serial.print(p - heap_end);
  Serial.println(" bytes");

  Serial.printf("heap size: %d\n", int(heap_end - heap_start));

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
  Serial.print(" (= ");              
  Serial.print(int(heap_end - (const uint8_t *)__malloc_sbrk_base));    
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

  Serial.printf("Calculated heap size: %d\n", int(heap_end - (const uint8_t *)&_end));    

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


void detect_stack_size(bool ok_to_blab = true) {
  caddr_t base_heap_end = _sbrk(0);

  const uint8_t * const heap_end = (const uint8_t *)base_heap_end;

  constexpr const int favorite_marker_byte = 0xC3;

  if (!get_sbrk_marker_byte_ref()) {

    // We're starting the app: fill the empty/unused space with a special marker byte
    // so we can detect actual stack usage!

    unsigned int histogram[256] = {0};
    int count = 0;
    unsigned short int low_idx;
    unsigned int low_val;
    const uint8_t *current_sstack = (const uint8_t *)get_current_stack_bottom_alloca();

    if (ok_to_blab) {
      report_current_stack_usage();
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
      if (i == 0)
        continue; // do not include byte value 0x00 in the scan: that value has been used to flag the 'fill/mark test has not been initialized yet' state!
      if (i == 0xFF)
        continue; // do not include byte value 0xFF in the scan: we don't want to see that one score a self-goal as it'll be ubiquitous in the Flash already...

      if (histogram[i] <= low_val) {
        low_idx = i;
        low_val = histogram[i];
      }
    }
    
    // check if our preferred 'magic value' has the lowest score: if it has, use it!
    if (histogram[favorite_marker_byte] == low_val)
      low_idx = favorite_marker_byte;

    get_sbrk_marker_byte_ref() = low_idx;

    constexpr int safety_margin = 16;
    
    // report:
    if (ok_to_blab) {
      for (unsigned short int i = 1; i < 256; i++) {
        Serial.print("[0x");
        Serial.print(i, 16);
        Serial.print("] = ");
        Serial.print(histogram[i]);
      
        if (histogram[i] == low_val) {
          Serial.print(" (*)");
        }

        Serial.println();
      }

      Serial.print("-------------------------\nlow @ 0x");
      Serial.print(get_sbrk_marker_byte_ref(), 16);
      Serial.println();
      Serial.print("-------------------------\nfilling empty RAM: length = ");
      Serial.print(current_sstack - heap_end - safety_margin);
      Serial.println();
    }

//#pragma GCC diagnostic push
// warning: array subscript -2 is outside array bounds of 'unsigned int [4]' [-Warray-bounds=]
//#pragma GCC diagnostic ignored "-Warray-bounds"

    auto* fill_top = current_sstack - safety_margin;   // a tiny safety margin...

//#pragma GCC diagnostic pop

    memset((void *)heap_end, get_sbrk_marker_byte_ref(), fill_top - heap_end);

    // init = true;
  }
  else {
    //assert(init);
    if (ok_to_blab) {
      report_current_stack_usage();
    }
  }
}


