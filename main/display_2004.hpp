#pragma once

#include "display.hpp"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <inttypes.h>

class display_2004 : display{
    private:
        LiquidCrystal_I2C lcd;
        void print_init();
    public:
        void draw_console_line(char* line);
        display_2004(TwoWire *bus, char adr) : display(bus,adr);
        ~display_2004() : ~display();
        void clear();
        void set_state(bool on);
        void draw_screen(int throttle, float steering, float desired_steering, int torgue[],bool gamepad, int throttle_gp, float steering_gp, float speed);
        void draw_menu(int options, char* option_name[], int highlight);
        void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        void draw_confirmation(char* text, int options, char* option_name[], int highlight);
}