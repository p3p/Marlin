/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef __PLAT_NATIVE_SIM__

#include <thread>
#include <atomic>

#include "application.h"
#include "execution_control.h"

#include "src/inc/MarlinConfig.h"


std::atomic_bool main_finished = false;

void HAL_idletask() {
  Kernel::yield();
}

extern void setup();
extern void loop();
void marlin_loop() {
  static bool initialised = false;
  if (!initialised) {
    initialised = true;
    #ifdef MYSERIAL0
      MYSERIAL0.begin(BAUDRATE);
      SERIAL_FLUSHTX();
    #endif
    HAL_timer_init();
    setup();
  } else loop();
}

void simulation_main() {
  pthread_setname_np(pthread_self(), "simulation_main");
  // Marlin Loop 500hz
  Kernel::Timers::timerInit(3, 1000000);
  Kernel::Timers::timerStart(3, 500);
  Kernel::Timers::timerEnable(3);
  Kernel::is_initialized(true);

  while(!main_finished) {
    try {
      Kernel::execute_loop();
    } catch (std::runtime_error& e) {
      // stack unrolled by exception in order to exit cleanly
      // todo: use a custom exception
      printf("Exception: %s\n", e.what());
      printf("Marlin thread terminated\n");
      main_finished = true;
    }
    std::this_thread::yield();
  }
}

// Main code
int main(int, char**) {
  Application app;

  std::thread simulation_loop(simulation_main);

  while (app.active) {
    app.update();
    app.render();
    std::this_thread::yield();
  }

  main_finished = true;
  Kernel::quit_requested = true;
  simulation_loop.join();

  return 0;
}

#endif // __PLAT_NATIVE_SIM__
