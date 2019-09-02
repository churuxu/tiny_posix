#include "tiny_posix.h"



extern void example_led();
extern void example_key();
extern void example_uart();


int main(){
    printf("hello world\n");
    //example_led();
    example_key();
    //example_uart();

    return 0;
}

