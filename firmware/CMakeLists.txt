# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)
 
# Set the name and version of the project
project(GPSDO VERSION 1.0.0)

# Initalise the SDK
pico_sdk_init()


# Link the Project to a source file (step 4.6)
add_executable(GPSDO ${CMAKE_CURRENT_LIST_DIR}/main.c  ${CMAKE_CURRENT_LIST_DIR}/si5351.c)     

# generate C header for the PIO programs
pico_generate_pio_header(GPSDO ${CMAKE_CURRENT_LIST_DIR}/counter.pio)
pico_generate_pio_header(GPSDO ${CMAKE_CURRENT_LIST_DIR}/pps.pio)


# Link the Project to an extra library (pico_stdlib)
target_link_libraries(GPSDO pico_stdlib hardware_timer hardware_i2c hardware_pwm hardware_gpio hardware_irq hardware_pio hardware_dma hardware_clocks)
 
# Enable USB, UART output
pico_enable_stdio_usb(GPSDO 1)
pico_enable_stdio_uart(GPSDO 0)
 
# Enable extra outputs (SWD?)
pico_add_extra_outputs(GPSDO)