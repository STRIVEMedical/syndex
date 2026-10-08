# SynDex base: Raspberry Pi 5 program (C++17, CMake).
#
# Build (from base/):   cmake -B build && cmake --build build
# Executables:          syndex_base       (the base program, from src/)
#                       syndex_calibrate  (stereo calibration tool, from tools/calibrate.cpp)
#
# OpenCV and libcamera are optional for now: if they are missing, configuration only warns, so the
# skeleton still configures on a machine without them (e.g. a laptop). Install them on the Pi with
# the packages listed in SETUP.md (libopencv-dev, libcamera-dev, pkg-config).

cmake_minimum_required(VERSION 3.16)
project(syndex_base LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Optional dependencies (warn, don't fail) ------------------------------------------------
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
  message(WARNING "OpenCV not found - LED detection / stereo code cannot use it yet. "
                  "On the Pi: sudo apt install libopencv-dev")
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA QUIET IMPORTED_TARGET libcamera)
endif()
if(NOT LIBCAMERA_FOUND)
  message(WARNING "libcamera not found (needs pkg-config and libcamera-dev) - camera capture cannot "
                  "use it yet. On the Pi: sudo apt install pkg-config libcamera-dev")
endif()

# TODO: link BlueZ (libbluetooth, e.g. pkg_check_modules(BLUEZ bluez)) for the Bluetooth serial
# Unity link once unity_link.cpp is implemented. Not added as a dependency yet.

# ---- Sources ---------------------------------------------------------------------------------
set(SYNDEX_BASE_SOURCES
  src/main.cpp
  src/camera.cpp
  src/led_detect.cpp
  src/stereo.cpp
  src/pose.cpp
  src/handle_link.cpp
  src/unity_link.cpp
  src/faults.cpp
  ../shared/serial_framing.cpp   # base only: serial framing for the Bluetooth Unity link

  # Pending Pi port: moved from V1 verbatim, but they still depend on Arduino/ODrive/joint/USB
  # code that does not exist on the Pi, so they cannot compile yet. Uncomment each one once it has
  # been ported.
  # src/states.cpp
  # src/status_leds.cpp
)

add_executable(syndex_base ${SYNDEX_BASE_SOURCES})
add_executable(syndex_calibrate tools/calibrate.cpp)

foreach(tgt syndex_base syndex_calibrate)
  target_include_directories(${tgt} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared)

  if(MSVC)
    target_compile_options(${tgt} PRIVATE /W4)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra)
  endif()

  if(OpenCV_FOUND)
    target_include_directories(${tgt} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${tgt} PRIVATE ${OpenCV_LIBS})
  endif()
endforeach()

# Camera capture is only needed by the base program for now.
if(LIBCAMERA_FOUND)
  target_link_libraries(syndex_base PRIVATE PkgConfig::LIBCAMERA)
endif()
