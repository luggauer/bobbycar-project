#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <inttypes.h>
#include "display.hpp"

class display_oled : public display{
    private:
        Adafruit_SSD1306 oled;
        bool set_state(STATES_OF_DISPLAY hstate);
        void draw_line(const char* in, int y);
        bool set_state(STATES_OF_DISPLAY hstate);
    public:
        void draw_console_line(char* line);
        display_oled(TwoWire *bus, char adr, uint8_t width, uint8_t height);
        ~display_oled();
        void clear();
        void draw_screen(int throttle, float steering, float desired_steering, int torgue[],bool gamepad, int throttle_gp, float steering_gp, float speed);
        void draw_menu(int options, char* option_name[], int highlight);
        void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        void draw_confirmation(char* text, int options, char* option_name[], int highlight);
};
