#
# PlatformIO pre: script for simulator builds
#

# Get the environment thus far for the build
Import("env")

#print(env.Dump())

#
# Give the binary a distinctive name
#

env['PROGNAME'] = "MarlinSimulator"

#
# If Xcode is installed add the path to its Frameworks folder,
# or if Mesa is installed try to use its GL/gl.h.
#

import sys
import subprocess
if sys.platform == 'darwin':

  #
  # Silence half of the ranlib warnings. (No equivalent for 'ARFLAGS')
  #
  env['RANLIBFLAGS'] += [ "-no_warning_for_no_symbols" ]

  # Default paths for Xcode and a lucky GL/gl.h dropped by Mesa
  xcode_path = "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks"
  mesa_path = "/opt/local/include/GL/gl.h"

  import os.path

  if os.path.exists(xcode_path):

    env['BUILD_FLAGS'] += [ "-F" + xcode_path ]
    print("Using OpenGL framework headers from Xcode.app")

  elif os.path.exists(mesa_path):

    env['BUILD_FLAGS'] += [ '-D__MESA__' ]
    print("Using OpenGL header from", mesa_path)

  else:

    print("\n\nNo OpenGL headers found. Install Xcode for matching headers, or use 'sudo port install mesa' to get a GL/gl.h.\n\n")

    # Break out of the PIO build immediately
    sys.exit(1)

elif sys.platform == 'linux':
  try:
    env['BUILD_FLAGS'] += [subprocess.run(['sdl2-config', '--cflags'], stdout=subprocess.PIPE).stdout.decode('utf-8').strip()]
  except:
    print("'sdl2-config' not in PATH")

elif sys.platform == 'win32':
  import os
  prefix = "C:\\"
  for path in os.environ['PATH'].split(';'):
    if 'msys64' in path:
      prefix = path.split('msys64')[0]
  bash_exe = os.path.join(prefix, 'msys64\\usr\\bin\\bash.exe')
  new_env = os.environ.copy()
  new_env['PATH'] += os.path.join(prefix, 'msys64\\usr\\bin')
  try:
    current_sdl_prefix = subprocess.run([bash_exe, 'sdl2-config', '--prefix'], stdout=subprocess.PIPE, env=new_env).stdout.decode('utf-8').strip()
    env['BUILD_FLAGS'] += [os.path.normpath(subprocess.run([bash_exe, 'sdl2-config', '--prefix={}{}'.format(os.path.join(prefix, "msys64"), current_sdl_prefix),'--cflags'], stdout=subprocess.PIPE, env=new_env).stdout.decode('utf-8').strip()).replace('\\', '\\\\')]
  except:
    print("'sdl2-config' not in PATH")

env.AddCustomTarget("upload", "$BUILD_DIR/${PROGNAME}", "$BUILD_DIR/${PROGNAME}")
