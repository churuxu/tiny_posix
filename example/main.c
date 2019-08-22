#include "tiny_posix.h"
#include "hal_config.h"



int main(){
    int led = GPIO_PIN_FD(PORTA, PIN1);
    tiny_posix_init();

    hal_config();

    while(1){
        gpio_set(led);
        sleep(1);
        gpio_reset(led);
        sleep(1);
    }

    return 0;
}


