#include "tiny_posix.h"
#include "hal_config.h"



int main(){
    int led = LED;
    //int port = SERIAL_PORT;

    tiny_posix_init();  

    while(1){
        gpio_set(led);
        sleep(1);
        gpio_reset(led);
        sleep(1);
        //uart_write(port, "SOpen_beepE", 11);
        //usleep(100000);
        //uart_write(port, "SClose_beepE", 12);
    }

    return 0;
}


