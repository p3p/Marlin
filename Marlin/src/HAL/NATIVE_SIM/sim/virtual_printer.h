#pragma once

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

class VirtualPrinter {
public:
  struct Component {
    Component() : identifier("UnnamedComponent") {
      printf("VirtualPrinter::Component[%s]: contruct\n", identifier.c_str());
    }
    Component(std::string identifier) : identifier(identifier) {
      printf("VirtualPrinter::Component[%s]: contruct\n", identifier.c_str());
    }
    virtual ~Component() {
      printf("VirtualPrinter::Component[%s]: decontruct\n", identifier.c_str());
    }

    virtual void update() {};
    virtual void ui_init() {};
    virtual void ui_widget() {};

    const std::string identifier;
  };

  void update() {
    for(auto const& it : components) it.second->update();
  }

  void ui_widgets();

  void build();
  void update_kinematics();

  template<typename T, class... Args>
  auto add_component(std::string name, Args&&... args) {
    auto component = std::make_shared<T>(args...);
    components[name] = component;
    return component;
  }

  template<typename T>
  auto get_component(std::string name) {
    return std::static_pointer_cast<T>(components[name]);
  }

  glm::vec4 effector_pos{};
  std::function<void(glm::vec4)> on_kinematic_update;

private:
  std::map<std::string, std::shared_ptr<Component>> components;
  std::vector<std::shared_ptr<Component>> steppers;
};
