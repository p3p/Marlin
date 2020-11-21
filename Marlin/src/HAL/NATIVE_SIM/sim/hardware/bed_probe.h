#pragma once

#include "Gpio.h"
#include "print_bed.h"

class BedProbe {
public:
  BedProbe(pin_type probe, glm::vec3 offset, glm::vec4& position, PrintBed& bed) : offset(offset), position(position), bed(bed) {
    Gpio::attach(probe, [this](GpioEvent& event){ this->interrupt(event); });
  }

  void interrupt(GpioEvent& event) {
    Gpio::set_pin_value(event.pin_id, position.y < bed.calculate_z({position.x + offset.x, (-position.z) + offset.y}) - offset.z);
  }

  glm::vec3 offset;
  glm::vec4& position;
  PrintBed& bed;
};
