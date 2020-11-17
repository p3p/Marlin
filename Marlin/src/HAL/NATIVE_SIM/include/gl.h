#pragma once

#include <GL/glew.h>
#if defined(__APPLE__) && !defined(__MESA__)
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif
