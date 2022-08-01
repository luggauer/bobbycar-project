#include "display.hpp"

display::display(TwoWire *bus, char adr){
    i2c_bus = bus;
    address = adr;
}

display::~display(){
    
}

STATES_OF_DISPLAY display::get_state(){
    return state;
}

return display::set_state(STATES_OF_DISPLAY hstate){
    if(state != hstate){
        clear();
        state = hstate;
        return true;
    }
    else{
        return false;
    }
}