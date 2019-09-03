#include "tiny_posix.h"



extern void example_led();
extern void example_key();
extern void example_key_nb();
extern void example_uart();
extern void example_uart_nb();

int main(){
    printf("hello world\n");
    //example_led();
    //example_key();
    example_key_nb();
    //example_uart();
    //example_uart_nb();


    return 0;
}

