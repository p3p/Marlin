#ifdef __PLAT_NATIVE_SIM__

#include <SDL2/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl.h>

#include "user_interface.h"
#include "application.h"

Application::Application() {
  sim.vis.create();

  user_interface.addElement<SerialMonitor>("Serial Monitor");
  //user_interface.addElement<TextureWindow>("Controller Display", sim.display.texture_id, (float)sim.display.width / (float)sim.display.height, [this](UiWindow* window){ this->sim.display.ui_callback(window); });
  user_interface.addElement<StatusWindow>("Status", &clear_color, [this](UiWindow* window){ this->sim.ui_info_callback(window); });
  user_interface.addElement<UiWindow>("Components", [this](UiWindow* window){ this->sim.testPrinter.ui_widgets(); });
  user_interface.addElement<Viewport>("Viewport", [this](UiWindow* window){ this->sim.vis.ui_viewport_callback(window); });
  //user_interface.addElement<GraphWindow>("graphs", sim.display.texture_id, 128.0 / 64.0, std::bind(&Simulation::ui_callback, &sim, std::placeholders::_1));

  user_interface.addElement<UiWindow>("Simulation", [this](UiWindow* window){
    //Simulation Time
    uint64_t time_source = Kernel::SimulationRuntime::nanos();
    uint64_t hours = (time_source / (Kernel::TimeControl::ONE_BILLION * 60 * 60)) ;
    uint64_t remainder = (time_source % (Kernel::TimeControl::ONE_BILLION * 60 * 60));
    uint64_t mins = (remainder / (Kernel::TimeControl::ONE_BILLION * 60));
    remainder = (remainder % (Kernel::TimeControl::ONE_BILLION * 60));
    uint64_t seconds = remainder / (Kernel::TimeControl::ONE_BILLION);
    remainder = remainder % (Kernel::TimeControl::ONE_BILLION);
    ImGui::Text("%02ld:%02ld:%02ld.%09ld", hours, mins, seconds, remainder); //TODO: work around cross platform format string differences

    // Simulation Control
    auto ui_realtime_scale = Kernel::TimeControl::realtime_scale.load();
    ImGui::PushItemWidth(-1);
    ImGui::SliderFloat("##SimSpeed", &ui_realtime_scale, 0.0f, 100.0f, "%.3f", 3.0f);
    ImGui::PopItemWidth();
    if (ImGui::Button("Pause")) ui_realtime_scale = 0.0f;
    ImGui::SameLine();
    if (ImGui::Button("Realtime")) ui_realtime_scale = 1.0f;
    ImGui::SameLine();
    if (ImGui::Button("Max")) ui_realtime_scale = 100.0f;
    ImGui::SameLine();
    if (ImGui::Button("Break")) Kernel::execution_break();
    Kernel::TimeControl::realtime_scale.store(ui_realtime_scale);
  });

}

Application::~Application() {

}

void Application::update() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);

    switch (event.type) {
      case SDL_DROPFILE: {
        char *dropped_filedir = event.drop.file;
        input_file.open(dropped_filedir);
        SDL_free(dropped_filedir);    // Free dropped_filedir memory
      } break;

      case SDL_WINDOWEVENT:
        if (event.window.event != SDL_WINDOWEVENT_CLOSE || event.window.windowID != SDL_GetWindowID((SDL_Window*)window.getHandle()))
          break;

      case SDL_QUIT: active = false; break;
    }
  }

  // File read into serial port
  if (input_file.is_open() && usb_serial.receive_buffer.free()) {
    uint8_t buffer[HalSerial::receive_buffer_size]{};
    auto count = input_file.readsome((char*)buffer, usb_serial.receive_buffer.free());
    usb_serial.receive_buffer.write(buffer, count);
    if (count == 0) input_file.close();
  }

  sim.update();
  user_interface.show();
}

void Application::render() {
  sim.vis.framebuffer->bind();
  glClearColor(clear_color.x, clear_color.y, clear_color.z, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  sim.vis.update();               // Update and render
  sim.vis.framebuffer->render();  // Render and unbind framebuffer

  user_interface.render();
  window.swap_buffers();
}

#endif // __PLAT_NATIVE_SIM__
