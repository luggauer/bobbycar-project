#include "display.hpp"

display::display(TwoWire *bus, char adr){
    i2c_bus = bus;
    address = adr;
    print_init();
}

display::~display(){
    
}