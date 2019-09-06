#include "tiny_posix.h"



extern void example_led();
extern void example_key();
extern void example_key_nb();
extern void example_uart();
extern void example_uart_nb();
extern void example_clock();
extern void example_tcp_client();


int main(){
    printf("hello world\n");
    //example_led();
    //example_key();
    //example_key_nb();
    //example_uart();
    //example_uart_nb();
    example_clock();
    //example_tcp_client();
    
    return 0;
}

