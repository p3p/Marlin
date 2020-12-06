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
#pragma once

#include <chrono>
#include "Gpio.h"
#include "../user_interface.h"
#include "src/core/types.h"

class LinearAxis: public Peripheral {
public:
  LinearAxis(const char* name, AxisEnum my_axis, uint32_t steps_per_unit, float max_position_logical, pin_type enable, pin_type dir, pin_type step, pin_type end_min, pin_type end_max, bool invert_travel = false, std::function<void(LinearAxis&)> update_position_callback =  [](LinearAxis&){});
  virtual ~LinearAxis();
  void update();
  void interrupt(GpioEvent& ev);

  float position_logical = 0;

  void ui_info_callback(UiWindow*) {
    ImGui::PushItemWidth(100);
    if (ImGui::InputFloat(min_pos_label.c_str(), &min_position_logical, 0.01f, 0.1f)) {
      min_position = min_position_logical * steps_per_unit;
    }
    ImGui::PopItemWidth();
  };

private:
  const char* name;
  AxisEnum my_axis;
  uint32_t steps_per_unit;
  int32_t max_position;
  pin_type enable_pin;
  pin_type dir_pin;
  pin_type step_pin;
  pin_type min_pin;
  pin_type max_pin;
  std::atomic_int32_t min_position;
  uint64_t last_update;
  int8_t invert_travel;
  std::atomic_int32_t position = 0;
  std::function<void(LinearAxis&)> update_position_callback;
  float min_position_logical = 0;
  float max_position_logical = 0;
  std::string min_pos_label;
  std::string max_pos_label;
};
