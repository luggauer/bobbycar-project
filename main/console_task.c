#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "c_data.h"
#include "string_tools.h"
#include "command_interpreter.h"

void bt_console_init(){}
void usb_console_init(){}

void tast_bt_console(void *ignore){}
void tast_usb_console(void *ignore){
    char buffer[128];
    c_data* buffer2 = NULL;
    c_data_spawn_ptr(buffer2);
    while(true){
        fscanf(stdin,"%s", buffer);
        exec(buffer,buffer2);
        if(buffer2->size != 0){
            c_data_extend_raw(buffer2, &endl4ptr, sizeof(endl4ptr));
            printf(buffer2->content);
        }
        c_data_set_size(buffer2, 0);
    }
    c_data_delete_ptr(buffer2);
}