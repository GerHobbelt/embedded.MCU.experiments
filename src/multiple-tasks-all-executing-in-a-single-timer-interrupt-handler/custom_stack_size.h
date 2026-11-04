
#pragma once


#if 0

// One way to size the stack: just drop some content in there and thus have the .stack segment grow!
// We're using GCC-specific functionality to place this into the .stack section, not where it would normally go.
// The linker script makes sure that the .stack section is at the right place.
__attribute__ ((section(".stack"), used))
static uint8_t StackSpace[9000];

#endif
