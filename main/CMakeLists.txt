set(srcs "main.c")

set(requires "bluepad32")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino" "espsoftwareserial" "Adafruit_SSD1306" "Adafruit-GFX-Library" "crc32" "sbus")
endif()

idf_component_register(SRCS "display_oled.cpp" "inputreader.c" "math_functions.c" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
