#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "inputreader.h"
#include "command_interpreter.h"

typedef bool (*func_ptr)(const char*);
typedef struct 
{
    const char* name;
    func_ptr exec;
} command;

static bool echo(const char* argv){
    printf("%s\n",argv);
    return true;
}
static bool print_help_of(const char* argv);

static bool cmd_set_steering(const char* argv){
    return true;
}

static bool cmd_set_throttle(const char* argv){
    return true;
}


static bool cmd_get_steering(const char* argv){
    printf("S:%f\n",get_steering());
    return true;
}

static bool cmd_get_throttle(const char* argv){
    printf("T:%i\n",get_throttle());
    return true;
}


static const command commands[] = {{"help",print_help_of},{"echo", echo},{"sets",cmd_set_steering},{"sett",cmd_set_throttle},{"gets",cmd_get_steering},{"gett",cmd_get_throttle},{"exec",exec}};

static bool print_help_of(const char* argv){
    printf("help:\n");
    for(int i = 0;i < (sizeof(commands)/sizeof(command));i++)
        printf("%s\n",commands[i].name);
    return true;
}

void exec(const char* exec){
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
            if(commands[i].exec(argv))
                printf("success");
            else
                printf("fail");
            break;
        }
    free(tmp);
}