# Adafruit Bus IO Library
# https://github.com/adafruit/Adafruit_BusIO
# MIT License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "src/SBUS2.cpp" "src/SBUS_usart_ESP32.cpp" 
                       INCLUDE_DIRS "src"
                       REQUIRES arduino)

project(sbus2)
