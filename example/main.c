#include "tiny_posix.h"
#include "hal_config.h"


static int leds_[] = {
    LED1,
#ifdef LED2
    LED2,
#endif    
#ifdef LED3
    LED3,
#endif 
};

void loop_leds(){
    static int on = 1;
    static int cur = 0;    
    if(on){
        gpio_set(leds_[cur]);
    }else{
        gpio_reset(leds_[cur]);
    }
    cur ++;
    if(cur==(sizeof(leds_)/4)){
        cur = 0;
        on = (on == 0);
    }
}

void on_key(){
    gpio_set(LED1);
}

int main(){
    char buf[64];
    int port1 = SERIAL1;
    int port2 = SERIAL2;
    int ret;

    tiny_posix_init(); 

    gpio_set_irq(KEY1, on_key);
    
    while(1){
        loop_leds();
                
        uart_write(port1, "hello", 5);        
        
        ret = uart_read(port2, buf, 64);
        if(ret>0){
            uart_write(port2, buf, ret);
        }
    }

    return 0;
}


