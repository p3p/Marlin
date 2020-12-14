#include <imgui.h>

#include "hardware/Heater.h"
#include "hardware/print_bed.h"
#include "hardware/LinearAxis.h"
#include "hardware/print_bed.h"
#include "hardware/bed_probe.h"
#include "hardware/ST7796Device.h"
#include "hardware/HD44780Device.h"
#include "hardware/ST7920Device.h"
#include "hardware/SDCard.h"
#include "hardware/W25QxxDevice.h"
#include "hardware/FilamentRunoutSensor.h"

#include "virtual_printer.h"

#include "src/inc/MarlinConfig.h"

constexpr uint32_t steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

void VirtualPrinter::update_kinematics() {
  effector_pos = glm::vec4{
    std::static_pointer_cast<LinearAxis>(linear_axis[0])->position_logical,
    std::static_pointer_cast<LinearAxis>(linear_axis[2])->position_logical,
    std::static_pointer_cast<LinearAxis>(linear_axis[1])->position_logical * -1.0f,
    std::static_pointer_cast<LinearAxis>(linear_axis[3])->position_logical
  };

  on_kinematic_update(effector_pos);
}

void VirtualPrinter::build() {
  linear_axis.push_back(add_component<LinearAxis>("X Axis", steps_per_unit[0], X_BED_SIZE, X_ENABLE_PIN, X_DIR_PIN, X_STEP_PIN, X_MIN_PIN, X_MAX_PIN, INVERT_X_DIR, [this](){this->update_kinematics();}));
  linear_axis.push_back(add_component<LinearAxis>("Y Axis", steps_per_unit[1], Y_BED_SIZE, Y_ENABLE_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_MIN_PIN, Y_MAX_PIN, INVERT_Y_DIR, [this](){this->update_kinematics();}));
  linear_axis.push_back(add_component<LinearAxis>("Z Axis", steps_per_unit[2], Z_MAX_POS, Z_ENABLE_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_MIN_PIN, Z_MAX_PIN, INVERT_Z_DIR, [this](){this->update_kinematics();}));
  linear_axis.push_back(add_component<LinearAxis>("E Axis", steps_per_unit[3], 0, E0_ENABLE_PIN, E0_DIR_PIN, E0_STEP_PIN, P_NC, P_NC, INVERT_E0_DIR, [this](){this->update_kinematics();}));

  auto print_bed = add_component<PrintBed>("Print Bed", glm::vec2{X_BED_SIZE, Y_BED_SIZE});

  #if HAS_BED_PROBE
    add_component<BedProbe>("Probe", Z_MIN_PROBE_PIN, glm::vec3 NOZZLE_TO_PROBE_OFFSET, effector_pos, *print_bed);
  #endif

  add_component<Heater>("Hotend Heater", HEATER_0_PIN, TEMP_0_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
  add_component<Heater>("Bed Heater", HEATER_BED_PIN, TEMP_BED_PIN, heater_data{12, 1.2}, hotend_data{325, 824, 0.897}, adc_data{4700, 12});
  #if HAS_SPI_FLASH
    add_component<W25QxxDevice>("SPI Flash", SCK_PIN, MISO_PIN, MOSI_PIN, W25QXX_CS_PIN, SPI_FLASH_SIZE);
  #endif
  #ifdef SDSUPPORT
    add_component<SDCard>("SD Card", SCK_PIN, MISO_PIN, MOSI_PIN, SDSS, SD_DETECT_PIN, SD_DETECT_STATE);
  #endif
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    add_component<FilamentRunoutSensor>("Filament Runout Sensor", FIL_RUNOUT1_PIN, FIL_RUNOUT_STATE);
  #endif

  #if ANY(TFT_COLOR_UI, TFT_CLASSIC_UI, TFT_LVGL_UI)
    add_component<ST7796Device>("ST7796Device Display", SCK_PIN, MISO_PIN, MOSI_PIN, TFT_CS_PIN, TOUCH_CS_PIN, TFT_DC_PIN, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #elif defined(HAS_MARLINUI_HD44780)
    add_component<HD44780Device>("HD44780Device Display", LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #elif defined(HAS_MARLINUI_U8GLIB)
    add_component<ST7920Device>("ST7920Device Display", LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #endif

  for(auto const& component : components) component.second->ui_init();

  update_kinematics();
}

void VirtualPrinter::ui_widgets() {
  for(auto const& it : components) {
    if (ImGui::CollapsingHeader(it.first.c_str())) {
      ImGui::PushID(it.first.c_str());
      it.second->ui_widget();
      ImGui::PopID();
    }
  }
}