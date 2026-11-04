
#undef USE_SERIAL  

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




static void setup_port_C() {

  constexpr static const uint32_t pinmask = PIO_PC1 | 0b11111111000011111110;

  pmc_enable_periph_clk(ID_PIOC);                                  // To use peripheral, we must enable clock distribution to it

  PIO_SetPeripheral(PIOC, PIO_OUTPUT_0, pinmask);
  PIO_Configure(PIOC, PIO_OUTPUT_0, pinmask, PIO_DEFAULT);
  PIO_SetOutput(PIOC, pinmask, 0 /* uint32_t dwDefaultValue */, 0 /* uint32_t dwMultiDriveEnable */, 0 /* uint32_t dwPullUpEnable */ );

  volatile auto dummy = PIOC->PIO_ISR;  // dummy read to clear any interrupt glitches during setup
  (void)dummy;

  //auto pinwpstat = PIOC->PIO_WPSR;

  uint32_t pinstatus = PIO_GetOutputDataStatus(PIOC, pinmask);
  (void)pinstatus;

  PIO_Clear(PIOC, pinmask);
  uint32_t pinval0 = PIO_Get(PIOC, PIO_OUTPUT_0, pinmask);
  (void)pinval0;
}

// initialize digital pin LED_BUILTIN as an output.
static void setup_port_B() {
  
  constexpr static const uint32_t pinmask = PIO_PB27;

  pmc_enable_periph_clk(ID_PIOB);                                  // To use peripheral, we must enable clock distribution to it

  //PIO_DisableInterrupt(PIOB, PIO_PB27B_TIOB0);
  //PIOB->PIO_IDR |= PIO_PB27B_TIOB0;
  //PIOB->PIO_IER &= ~PIO_PB27B_TIOB0;

  //PIO_PullUp(PIOB, PIO_PB27B_TIOB0, false /* const uint32_t dwPullUpEnable */ );
  //PIOB->PIO_PUER &= ~PIO_PB27B_TIOB0;
  //PIOB->PIO_PUDR |= PIO_PB27B_TIOB0;

  //PIO_SetDebounceFilter(PIOB, PIO_PB27B_TIOB0, 0 /* const uint32_t dwCuttOff */ );

  PIO_SetPeripheral(PIOB, PIO_OUTPUT_0, pinmask);
  PIO_Configure(PIOB, PIO_OUTPUT_0, pinmask, PIO_DEFAULT);
  //PIO_SetInput(PIOB, PIO_PB27B_TIOB0, PIO_DEFAULT);
  PIO_SetOutput(PIOB, pinmask, 0 /* uint32_t dwDefaultValue */, 0 /* uint32_t dwMultiDriveEnable */, 0 /* uint32_t dwPullUpEnable */ );

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

  volatile auto dummy = PIOB->PIO_ISR;  // dummy read to clear any interrupt glitches during setup
  (void)dummy;

  //auto pinwpstat = PIOB->PIO_WPSR;

  uint32_t pinstatus = PIO_GetOutputDataStatus(PIOB, pinmask);
  (void)pinstatus;

  PIO_Clear(PIOB, pinmask);
  uint32_t pinval0 = PIO_Get(PIOB, PIO_OUTPUT_0, pinmask);
  (void)pinval0;
}




static volatile bool debug_print_available = false;


/*
 * Cortex-M3 Systick IT handler
 */
#if 0
extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}
#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
//void initVariant() __attribute__((weak));
void initVariant() { }


//void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
//void UART_Handler(void)  __attribute__((weak));
void UART_Handler(void)
{
#if defined(USE_SERIAL)
  Serial.IrqHandler();
#endif
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
//void serialEvent1() __attribute__((weak));
void serialEvent1() { }
//void serialEvent2() __attribute__((weak));
void serialEvent2() { }
//void serialEvent3() __attribute__((weak));
void serialEvent3() { }
//void serialEvent4() __attribute__((weak));
void serialEvent4() { }

// IT handlers
void USART0_Handler(void)
{
#if defined(USE_SERIAL)
  Serial1.IrqHandler();
#endif  
}

void USART1_Handler(void)
{
#if defined(USE_SERIAL)
  Serial2.IrqHandler();
#endif
}

void USART3_Handler(void)
{
#if defined(USE_SERIAL)
  Serial3.IrqHandler();
#endif
}

void USART2_Handler(void)
{
#if defined(USE_SERIAL)
  Serial4.IrqHandler();
#endif
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
#if 0
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
  if (Serial4.available()) serialEvent4();
#endif
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init_alt( void )
{
#if 01
  SystemInit();

  // Set Systick to (1000 / SYSTICK_FREQUENCY) ms interval, common to all SAM3 variants
  if (TimeTick_Configure(SystemCoreClock))
  {
    // Capture error
    while (true);
  }
#endif

#if 0
  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);
#endif

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  PIOC->PIO_OWER = 0xFFFFFFFF;
  PIOD->PIO_OWER = 0xFFFFFFFF;

#if defined(USE_SERIAL)
  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pPort,
    g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin,
    g_APinDescription[PINS_UART].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pPort,
    g_APinDescription[PINS_USART0].ulPinType,
    g_APinDescription[PINS_USART0].ulPin,
    g_APinDescription[PINS_USART0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pPort,
    g_APinDescription[PINS_USART1].ulPinType,
    g_APinDescription[PINS_USART1].ulPin,
    g_APinDescription[PINS_USART1].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART3].pPort,
    g_APinDescription[PINS_USART3].ulPinType,
    g_APinDescription[PINS_USART3].ulPin,
    g_APinDescription[PINS_USART3].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART2].pPort,
    g_APinDescription[PINS_USART2].ulPinType,
    g_APinDescription[PINS_USART2].ulPin,
    g_APinDescription[PINS_USART2].ulPinConfiguration);    

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  // Initialize CAN pins
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pPort,
    g_APinDescription[PINS_CAN0].ulPinType,
    g_APinDescription[PINS_CAN0].ulPin,
    g_APinDescription[PINS_CAN0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pPort,
    g_APinDescription[PINS_CAN1].ulPinType,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);
#endif

#if 0
  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
  analogOutputInit();
#endif

  // Initialize C library and run C++ constructors
  __libc_init_array();
}

#ifdef __cplusplus
}
#endif


/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	// Initialize watchdog
	watchdogSetup();

	init_alt();

	//initVariant();

	delay(1);

#if defined(USBCON) && defined(USE_SERIAL)
	USBDevice.attach();
#endif

  setup();
  
  for(;;) {
    loop();
  }

  return 0;
}

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef USE_SERIAL
	Serial.begin(250000);

  debug_print_available = true;

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
#endif

  setup_port_B();
  setup_port_C();
}

// the loop function runs over and over again forever
void loop() {
  static uint32_t time_of_last_user_action = 0;
  // reset the 'interactivity is happening' flag/time to 'inactive' when 8 seconds have passed since the last user activity:
  if (time_of_last_user_action && GetTickCount() > time_of_last_user_action + SYSTICK_MS_TO_TICKS(9000)) {
    time_of_last_user_action = 0;
  }

  //digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  PIO_Set(PIOB, PIO_PB27);
  PIO_Set(PIOC, 0x000FF);
  delay(150);
  //digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  PIO_Clear(PIOB, PIO_PB27);
  PIO_Clear(PIOC, 0xFFFFF);
  delay(150);
}

