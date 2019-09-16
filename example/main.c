#include "tiny_posix.h"



extern void example_led();
extern void example_key();
extern void example_key_nb();
extern void example_uart();
extern void example_uart_nb();
extern void example_rtc();
extern void example_clock();
extern void example_tcp_client();
extern void example_tcp_server();
extern void example_file();

int main(){
    int i = 0;

   //test_win32();
    //example_led();
    //example_key();
    //example_key_nb();
    //example_uart();
    //example_uart_nb();
    //example_rtc();
    //example_clock();
    //example_tcp_client();
    example_tcp_server();
    //example_file();

    while(1){
        i++;
        printf("hello world %d\n", i);
        sleep(1);
    }

    return 0;
}

