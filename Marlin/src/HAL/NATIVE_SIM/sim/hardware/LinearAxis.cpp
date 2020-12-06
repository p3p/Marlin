/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

#include "../execution_control.h"
#include <random>
#include <stdio.h>
#include "LinearAxis.h"

LinearAxis::LinearAxis(const char* name, AxisEnum my_axis, uint32_t steps_per_unit, float max_position_logical, pin_type enable, pin_type dir, pin_type step, pin_type end_min, pin_type end_max, bool invert_travel, std::function<void(LinearAxis&)> update_position_callback) :
  name(name), my_axis(my_axis), steps_per_unit(steps_per_unit), max_position(max_position_logical * steps_per_unit), enable_pin(enable), dir_pin(dir), step_pin(step), min_pin(end_min), max_pin(end_max), invert_travel(invert_travel ? -1 : 1), update_position_callback(update_position_callback)
 {
  min_position = 0;
  // max_position = (200 * steps_per_unit) + min_position;
  position = 0;
  if (max_position > 0)
    position = rand() % ((max_position - 40) - min_position) + (min_position + 20);

  position_logical = position / (float)steps_per_unit;
  last_update = Kernel::TimeControl::getTicks();

  min_pos_label = std::string(name) + " Min Offset";
  max_pos_label = std::string(name) + " Max Offset";

  Gpio::set_pin_value(min_pin, (position < min_position));
  Gpio::set_pin_value(max_pin, (position > max_position));
  Gpio::attach(step_pin, [this](GpioEvent &event){ this->interrupt(event); });
  Gpio::attach(min_pin, [this](GpioEvent &event){ this->interrupt(event); });
  Gpio::attach(max_pin, [this](GpioEvent &event){ this->interrupt(event); });
}

LinearAxis::~LinearAxis() {

}

void LinearAxis::update() {

}

void LinearAxis::interrupt(GpioEvent& ev) {
  Gpio::set_pin_value(min_pin, (position < min_position));
  Gpio::set_pin_value(max_pin, (position > max_position));

  if (ev.pin_id == step_pin && !Gpio::get_pin_value(enable_pin)){
    if (ev.event == GpioEvent::RISE) {
      last_update = ev.timestamp;
      position += (Gpio::get_pin_value(dir_pin) > 0 ? 1 : -1) * invert_travel;
      position_logical = position / (float)steps_per_unit;
      update_position_callback(*this);
    }
  }
}

#endif // __PLAT_NATIVE_SIM__
