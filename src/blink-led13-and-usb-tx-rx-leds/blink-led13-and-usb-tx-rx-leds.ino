#include <Arduino.h>

#undef USE_SERIAL

//extern WEAK const uint8_t SystemCoreTickFreqMultiplier = 1;

/*
The Arduino Due LEDs:

- the classical 'pin 13' Arduino LED is located at PB27 and is SET for ON.
- TXL = PA21 = Digital Pin 73 and is CLEARED for ON.
- RXL = PC30 = Digital Pin 72 and is CLEARED for ON.
*/


static void setup_PIO_port(int id_pio, Pio *pio, uint32_t pinmask) {

  pmc_enable_periph_clk(id_pio);                                  // To use peripheral, we must enable clock distribution to it

  PIO_SetPeripheral(pio, PIO_OUTPUT_0, pinmask);
  PIO_Configure(pio, PIO_OUTPUT_0, pinmask, PIO_DEFAULT);
  PIO_SetOutput(pio, pinmask, 0 /* uint32_t dwDefaultValue */, 0 /* uint32_t dwMultiDriveEnable */, 0 /* uint32_t dwPullUpEnable */ );

  volatile auto dummy = pio->PIO_ISR;  // dummy read to clear any interrupt glitches during setup
  (void)dummy;

  //auto pinwpstat = pio->PIO_WPSR;

  uint32_t pinstatus = PIO_GetOutputDataStatus(pio, pinmask);
  (void)pinstatus;

  PIO_Clear(pio, pinmask);
  uint32_t pinval0 = PIO_Get(pio, PIO_OUTPUT_0, pinmask);
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

void initUSB() {
#if defined(USBCON) && defined(USE_SERIAL)
	USBDevice.attach();
#endif
}

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

void init( void )
{
  SystemInit();

  // Set Systick to (1000 / SYSTICK_FREQUENCY) ms interval, common to all SAM3 variants
  if (TimeTick_Configure(SystemCoreClock))
  {
    // Capture error
    while (true);
  }

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


#if 0
/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	// Initialize watchdog
	watchdogSetup();

	init();

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
#endif

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef USE_SERIAL
	Serial.begin(250000);

  debug_print_available = true;

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
	Serial.println("BEGIN");
#endif

  setup_PIO_port(ID_PIOA, PIOA, PIO_PA21);
  setup_PIO_port(ID_PIOB, PIOB, PIO_PB27);
  setup_PIO_port(ID_PIOC, PIOC, PIO_PC30);
}

// the loop function runs over and over again forever
void loop() {
#ifdef USE_SERIAL
	Serial.println(SYSTICK_MS_TO_TICKS(10));
#endif
  //digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  PIO_Set(PIOB, PIO_PB27);
  delay(25);
  PIO_Clear(PIOA, PIO_PA21);
  delay(25);
  PIO_Clear(PIOC, PIO_PC30);
  delay(250);
  //digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  PIO_Set(PIOC, PIO_PC30);
  delay(25);
  PIO_Set(PIOA, PIO_PA21);
  delay(25);
  PIO_Clear(PIOB, PIO_PB27);
  delay(250);
}

