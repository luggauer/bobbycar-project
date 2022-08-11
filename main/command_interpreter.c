#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "c_data.h"
#include "inputreader.h"
#include "command_interpreter.h"

const char endl4ptr = '\0';
const char newl4ptr = '\n';

typedef bool (*func_ptr)(const char*, c_data*);
typedef struct 
{
    const char* name;
    func_ptr exec;
} command;

const char* inputs[] = {"ADC","Console", "Gamepad"};

static bool echo(const char* argv, c_data* out){
    c_data_extend_raw(out, argv, strlen(argv));
    c_data_extend_raw(out, &newl4ptr, sizeof(newl4ptr));
    return true;
}
static bool print_help_of(const char* argv, c_data* out);

static bool cmd_set_steering(const char* argv, c_data* out){
    return true;
}

static bool cmd_set_throttle(const char* argv, c_data* out){
    return true;
}


static bool cmd_get_steering(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "S:%f\n",get_steering());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_get_throttle(const char* argv, c_data* out){
    char buffer[20];
    sprintf(buffer, "T:%i\n",get_throttle());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}

static bool cmd_set_input(const char* argv, c_data* out){
    char buffer[20];
    for(int i = 0;i < (sizeof(inputs)/sizeof(char*));i++)
        if(strcmp(inputs[i],argv) == 0){
            set_input_src(i);
            return true;
        }
    return false;
}

static bool cmd_get_input(const char* argv, c_data* out){
    char buffer[20];
    //sprintf(buffer, "I:%s\n",get_throttle());
    c_data_extend_raw(out, buffer, strlen(buffer));
    return true;
}



static const command commands[] = {{"help",print_help_of},{"echo", echo},{"sets",cmd_set_steering},{"sett",cmd_set_throttle},{"gets",cmd_get_steering},{"gett",cmd_get_throttle},{"exec",exec}};

static bool print_help_of(const char* argv, c_data* out){
    printf("help:\n");
    for(int i = 0;i < (sizeof(commands)/sizeof(command));i++)
        printf("%s\n",commands[i].name);
    return true;
}

void exec(const char* exec, c_data* out){
    char* tmp = malloc(strlen(exec));
    strcpy(tmp,exec);
    char* argv = &tmp[strlen(exec)];
    for (int i=0; tmp[i]; i++)
        if(tmp[i]==' ' || tmp[i]=='\n' || tmp[i]=='\t'){
            tmp[i] = '\0';
            if(tmp[i+1])
                argv = &tmp[i+1];
            else
                argv = &tmp[i];
            break;
        }
    for(int i = 0;i < (sizeof(commands)/sizeof(command));i++)
        if(strcmp(commands[i].name,tmp) == 0){
            printf("execute %i: %s (%s) with %s\n",i,tmp,commands[i].name,argv);
            if(commands[i].exec(argv, out))
                printf("success");
            else
                printf("fail");
            break;
        }
    free(tmp);
}