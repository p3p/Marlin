#pragma once

#include "Gpio.h"
#include "../user_interface.h"

class FilamentRunoutSensor {
public:
  FilamentRunoutSensor(pin_type runout_pin, bool runtout_trigger_value) : runout_pin(runout_pin), runtout_trigger_value(runtout_trigger_value) {
    Gpio::attach(runout_pin, [this](GpioEvent& event){ this->interrupt(event); });
  }

  void interrupt(GpioEvent &ev) {
    if (ev.pin_id == runout_pin && ev.event == GpioEvent::GET_VALUE) {
      Gpio::set_pin_value(runout_pin, filament_present ? !runtout_trigger_value : runtout_trigger_value);
    }
  }

  void ui_info_callback(UiWindow*) {
    ImGui::Checkbox("Filament Present ", (bool*)&filament_present);
  }

private:
  pin_type runout_pin;
  bool runtout_trigger_value;
  bool filament_present = true;
};
