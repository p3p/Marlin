#pragma once

#include <glm/glm.hpp>

#include "Gpio.h"
#include "print_bed.h"

class PrintBed {
public:
  PrintBed(glm::vec2 size) : bed_size(size), bed_plane_center({size.x/2, size.y/2, 0}) {}

  void build_3point(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
    auto v1 = p2 - p1;
    auto v2 = p3 - p1;
    bed_plane_normal = glm::normalize(glm::cross(v1, v2));
    bed_plane_center.z = ((bed_plane_normal.x * (bed_plane_center.x - p1.x) + bed_plane_normal.y * (bed_plane_center.y - p1.y)) / (- bed_plane_normal.z)) + p1.z;
  }

  float calculate_z(glm::vec2 xy) {
    // bed_plane_normal.x * (x - p1.x) + bed_plane_normal.y * (y - p1.y) + bed_plane_normal.z * (z - p1.z) = 0
    // bed_plane_normal.x * (x - p1.x) + bed_plane_normal.y * (y - p1.y) = - bed_plane_normal.z * (z - p1.z)
    // ((bed_plane_normal.x * (x - p1.x) + bed_plane_normal.y * (y - p1.y)) / (- bed_plane_normal.z)) + p1.z  =  z
    return ((bed_plane_normal.x * (xy.x - bed_plane_center.x) + bed_plane_normal.y * (xy.y - bed_plane_center.y)) / (- bed_plane_normal.z)) + bed_plane_center.z;
  }

  glm::vec2 bed_size{200, 200};
  glm::vec3 bed_plane_center{100, 100, 0};
  glm::vec3 bed_plane_normal{0, 0, 1};
};
