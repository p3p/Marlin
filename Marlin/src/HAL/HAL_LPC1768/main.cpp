#ifdef TARGET_LPC1768

#include <LPC1768_PWM.h>
#include <usb/usb.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbcore.h>
#include <usb/cdc.h>
#include <usb/cdcuser.h>
#include <usb/mscuser.h>
#include <CDCSerial.h>
extern "C" {
  #include <debug_frmwrk.h>
}
#include "../../inc/MarlinConfig.h"
#include "HAL.h"
#include "HAL_timers.h"

extern uint32_t MSC_SD_Init(uint8_t pdrv);
extern "C" int isLPC1769();
extern "C" void disk_timerproc(void);

void SysTick_Callback() {
  disk_timerproc();
}

void HAL_init() {
  #if PIN_EXISTS(LED)
    SET_DIR_OUTPUT(LED_PIN);
    WRITE_PIN_CLR(LED_PIN);

    // MKS_SBASE has 3 other LEDs the bootloader uses during flashing. Clear them.
    SET_DIR_OUTPUT(P1_19);
    WRITE_PIN_CLR(P1_19);
    SET_DIR_OUTPUT(P1_20);
    WRITE_PIN_CLR(P1_20);
    SET_DIR_OUTPUT(P1_21);
    WRITE_PIN_CLR(P1_21);

    // Flash status LED 3 times to indicate Marlin has started booting
    for (uint8_t i = 0; i < 6; ++i) {
      TOGGLE(LED_PIN);
      delay(100);
    }
  #endif
  //debug_frmwrk_init();
  //_DBG("\n\nDebug running\n");
  // Initialise the SD card chip select pins as soon as possible
  #ifdef SS_PIN
    digitalWrite(SS_PIN, HIGH);
    pinMode(SS_PIN, OUTPUT);
  #endif
  #ifdef ONBOARD_SD_CS
    digitalWrite(ONBOARD_SD_CS, HIGH);
    pinMode(ONBOARD_SD_CS, OUTPUT);
  #endif
  USB_Init();                               // USB Initialization
  USB_Connect(TRUE);                        // USB Connect
  #ifdef USB_SD_ACCESS
    #if USB_SD_ACCESS != USB_SD_DISABLED
      MSC_SD_Init(0);                       // Enable USB SD card access
    #endif
  #endif
  const uint32_t usb_timeout = millis() + 2000;
  while (!USB_Configuration && PENDING(millis(), usb_timeout)) {
    delay(50);
    HAL_idletask();
    #if PIN_EXISTS(LED)
      TOGGLE(LED_PIN);     // Flash quickly during USB initialization
    #endif
  }

  #if NUM_SERIAL > 0
    MYSERIAL0.begin(BAUDRATE);
    #if NUM_SERIAL > 1
      MYSERIAL1.begin(BAUDRATE);
    #endif
    SERIAL_PRINTF("\n\necho:%s (%dMhz) Initialized\n", isLPC1769() ? "LPC1769" : "LPC1768", SystemCoreClock / 1000000);
    SERIAL_FLUSHTX();
  #endif

  HAL_timer_init();
  LPC1768_PWM_init();
}

#endif // TARGET_LPC1768
