#include "display.hpp"

display::display(TwoWire *bus, char adr){
    i2c_bus = bus;
    address = adr;
    print_init();
}

display::~display(){
    
}

STATES_OF_DISPLAY display::get_state(){
    return state;
}

void display::set_state(STATES_OF_DISPLAY hstate){
    if(state != hstate){
        clear();
        state = hstate;
    }
}