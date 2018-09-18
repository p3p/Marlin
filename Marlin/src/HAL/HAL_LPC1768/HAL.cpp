/* **************************************************************************

 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

#ifdef TARGET_LPC1768

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "../../../gcode/parser.h"
#include "../../sd/cardreader.h"
#include <usb/mscuser.h>
// U8glib required functions
extern "C" void u8g_xMicroDelay(uint16_t val) {
  DELAY_US(val);
}
extern "C" void u8g_MicroDelay(void) {
  u8g_xMicroDelay(1);
}
extern "C" void u8g_10MicroDelay(void) {
  u8g_xMicroDelay(10);
}
extern "C" void u8g_Delay(uint16_t val) {
  delay(val);
}
//************************//

// return free heap space
int freeMemory() {
  char stack_end;
  void *heap_start = malloc(sizeof(uint32_t));
  if (heap_start == 0) return 0;

  uint32_t result = (uint32_t)&stack_end - (uint32_t)heap_start;
  free(heap_start);
  return result;
}

int16_t PARSED_PIN_INDEX(const char code, const int16_t dval) {
  const uint16_t val = (uint16_t)parser.intval(code), port = val / 100, pin = val % 100;
  const  int16_t ind = (port < (NUM_DIGITAL_PINS >> 5) && (pin < 32))
                      ? GET_PIN_MAP_INDEX(port << 5 | pin) : -2;
  return ind > -2 ? ind : dval;
}

// HAL idle task
void HAL_idletask(void) {
  #if ENABLED(SHARED_SD_CARD)
    // If Marlin is using the SD card we need to lock it to prevent access from
    // a PC via USB.
    // Other HALs use IS_SD_PRINTING and IS_SD_FILE_OPEN to check for access but
    // this will not reliably detect delete operations. To be safe we will lock
    // the disk if Marlin has it mounted. Unfortuately there is currently no way
    // to unmount the disk from the LCD menu.
    // if (IS_SD_PRINTING || IS_SD_FILE_OPEN)
    if (card.cardOK)
      MSC_Aquire_Lock();
    else
      MSC_Release_Lock();
  #endif
  // Perform USB stack housekeeping
  MSC_RunDeferredCommands();
}

#endif // TARGET_LPC1768
