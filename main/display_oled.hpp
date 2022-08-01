#pragma once

#include "display.hpp"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <inttypes.h>

class display_oled : display{
    private:
        Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
        void print_init();
        void set_state(int hstate) : set_state(hstate);
    public:
        void draw_console_line(char* line);
        display_oled(TwoWire *bus, char adr, uint8_t width, uint8_t height) : display(bus,adr);
        ~display_oled() : ~display();
        void clear();
        void set_state(bool on);
        void draw_screen(int throttle, float steering, float desired_steering, int torgue[],bool gamepad, int throttle_gp, float steering_gp, float speed);
        void draw_menu(int options, char* option_name[], int highlight);
        void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        void draw_confirmation(char* text, int options, char* option_name[], int highlight);
}