#pragma once

#include <memory>
#include <atomic>
#include <glm/glm.hpp>

#include <imgui.h>

#include "Gpio.h"
#include "../virtual_printer.h"
#include "StepperDriver.h"

class KinematicSystem : public VirtualPrinter::Component {
public:
  KinematicSystem(std::vector<std::shared_ptr<VirtualPrinter::Component>>& steppers, std::function<void(glm::vec4)> on_kinematic_update) : VirtualPrinter::Component("Kinematic System"), steppers(steppers), on_kinematic_update(on_kinematic_update) {
    for (auto stepper : steppers) { std::static_pointer_cast<StepperDriver>(stepper)->step_callback = [this](){ this->kinematic_update(); }; }
    srand(time(0));
    origin.x = (rand() % (X_MAX_POS - X_MIN_POS)) + X_MIN_POS;
    origin.y = (rand() % (Y_MAX_POS - Y_MIN_POS)) + Y_MIN_POS;
    origin.z = (rand() % (Z_MAX_POS - Z_MIN_POS)) + Z_MIN_POS;
  }
  ~KinematicSystem() {}

  void ui_widget();
  void kinematic_update();

  glm::vec4 effector_position{}, stepper_position{};
  glm::vec3 origin{};
  std::vector<std::shared_ptr<VirtualPrinter::Component>>& steppers;
  std::function<void(glm::vec4)> on_kinematic_update;

};