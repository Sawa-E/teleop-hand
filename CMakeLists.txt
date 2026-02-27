cmake_minimum_required(VERSION 3.5)
project("TeleopHand")
set(TARGET_NAME "control")

include_directories(./inc/)
include_directories(./lib/)
include_directories(./lib/imgui)
add_subdirectory(lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")

option(USE_PCI_MODE "Enable PCI hardware mode" ON)
if(USE_PCI_MODE)
  add_definitions(-DPCI_MODE)
endif()

# if not defined then timer will not wait.
add_definitions(-DREALTIME)

add_executable(${TARGET_NAME}
  ./src/main.cc
  ./src/control_timer.cc
  ./src/thread_controller.cc
  ./src/system_controller.cc
  ./src/gui.cc
  ./src/controller.cc
)

if (UNIX)
  if(USE_PCI_MODE)
    target_link_libraries(${TARGET_NAME} GUI -lpthread -lGL -lrt -lglfw -lm -ldl stdc++fs -lpci)
  else()
    target_link_libraries(${TARGET_NAME} GUI -lpthread -lGL -lrt -lglfw -lm -ldl stdc++fs)
  endif()
endif()
