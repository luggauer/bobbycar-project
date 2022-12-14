#include "display_oled.hpp"
#include <stdlib.h>

display_oled::display_oled(TwoWire *bus, char adr, uint8_t width, uint8_t height) : display(bus,adr){
    bool dsp_connected;
    oled = Adafruit_SSD1306(width, height, bus);
    if(!(dsp_connected = oled.begin(SSD1306_SWITCHCAPVCC, adr)))
        printf("SSD1306 allocation failed");
    else
        oled.display();
}

void display_oled::draw_console_line(char* line){

}

void display_oled::draw_line(const char* in, int y) {
    oled.setCursor(0, y);     // Start at top-left corner

    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for(int16_t i=0; in[i]; i++)
        oled.write(in[i]);
    oled.display();
}

display_oled() :: ~display_oled(){

}
void display_oled::clear(){

}
void display_oled::draw_screen(int throttle, float steering, float desired_steering, int torgue[],bool gamepad, int throttle_gp, float steering_gp, float speed){
    char sprint_buffer[256];
    sprintf(sprint_buffer, "Throttle: %i\nSteering: %f\n%i  \t  %i\n%i  \t  %i\n%i: S%i B%i Sp:%f",throttle,steering*45/M_PI_4,torgue[0],torgue[1],torgue[2],torgue[3],gamepad,steering_gp,throttle_gp,speed);
    oled.clearDisplay();
    draw_line(sprint_buffer, 0);
}

void display_oled::set_state(STATES_OF_DISPLAY hstate){
    if(display::set_state(hstate)){
        switch (hstate)
        {
        case OFF:
            clear();
            break;

        case IDLE:
            break;
        
        case CONSOLE:
            clear();
        case USERINTERFACE:
            oled.setTextSize(1);      // Normal 1:1 pixel scale
            oled.setTextColor(SSD1306_WHITE); // Draw white text
            oled.cp437(true);         // Use full 256 char 'Code Page 437' font
        default:
            break;
        }
    return true;
  }
  else{
    return false;
  }
}

void display_oled::draw_menu(int options, char* option_name[], int highlight){

}

void display_oled::draw_menu_w_selection(int options, char* option_name[], int highlight, char* selection){

}

void display_oled::draw_confirmation(char* text, int options, char* option_name[], int highlight){

}