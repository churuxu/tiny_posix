#include "tiny_posix.h"
#include "hal_config.h"



void on_key(){
    gpio_set(LED1);
}

int main(){
       //int port = SERIAL_PORT;

    tiny_posix_init();  
    gpio_set_irq(KEY1, on_key);
    gpio_reset(LED1);

    while(1){
        //gpio_set(LED1);
        sleep(1);
        gpio_reset(LED1);
        //sleep(1);
        //uart_write(port, "SOpen_beepE", 11);
        //usleep(100000);
        //uart_write(port, "SClose_beepE", 12);
    }

    return 0;
}


