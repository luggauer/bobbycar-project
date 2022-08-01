#pragma once

#include <Wire.h>

typedef enum{IDLE, OFF, STATUS, CONSOLE} STATES_OF_DISPLAY;

class display{
    private:
        STATES_OF_DISPLAY state;
        char* buffer;
    protected:
        void set_state(STATES_OF_DISPLAY hstate);
        TwoWire *i2c_bus;
        char address;
        virtual void print_init();
    public:
        STATES_OF_DISPLAY get_state();
        virtual void draw_console_line(char* line);
        display(TwoWire *bus, char adr);
        virtual ~display();
        virtual void clear();
        virtual void set_state(bool on);
        virtual void draw_screen(int throttle, float steering, float desired_steering, int torgue[],bool gamepad, int throttle_gp, float steering_gp, float speed);
        virtual void draw_menu(int options, char* option_name[], int highlight);
        virtual void draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection);
        virtual void draw_confirmation(char* text, int options, char* option_name[], int highlight);
};
