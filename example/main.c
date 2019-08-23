#include "tiny_posix.h"
#include "hal_config.h"



int main(){
    int led = GPIO_FD(PORTA, PIN15);
    int port = UART_FD(2);

    tiny_posix_init();

    hal_config();

    while(1){
        gpio_set(led);
        sleep(1);
        gpio_reset(led);
        sleep(1);
        //uart_write(port, "hello", 5);
    }

    return 0;
}


