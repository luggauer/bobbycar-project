set(srcs "main.c")

set(requires "bluepad32")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino" "espsoftwareserial" "LiquidCrystal_I2C" "Adafruit_SSD1306" "Adafruit-GFX-Library" "sbus" "crc32" "Arduino-PID-Library" "memory_system_data")
endif()

idf_component_register(SRCS "string_tools.c" "command_interpreter.c" "console_task.c" "inputreader.c" "gamepad_manager.cpp" "adc1_read.c" "math_functions.c" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
