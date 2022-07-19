# Adafruit Bus IO Library
# https://github.com/adafruit/Adafruit_BusIO
# MIT License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "LiquidCrystal_I2C.cpp" 
                       INCLUDE_DIRS "."
                       REQUIRES arduino)

project(LiquidCrystal_I2C)
