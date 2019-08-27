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


#ifdef TEST_SPI1
void test_spi(){
    uint8_t cmd1[5] = {0x90,0,0,0,0};
    uint8_t cmd2[5] = {0x9f,0,0,0,0};
    uint8_t buf[8];
    int ret;
    gpio_reset(TEST_SPI1_NSS);
    //spi_write(TEST_SPI1, cmd, 1);
    //ret = spi_read(TEST_SPI1, buf, 16);
    ret = spi_io(TEST_SPI1, cmd1, buf, 5);    
    gpio_set(TEST_SPI1_NSS);
    if(ret>0){        
        uart_write(SERIAL2, buf, ret);
    }

    sleep(1);

    gpio_reset(TEST_SPI1_NSS);    
    ret = spi_io(TEST_SPI1, cmd2, buf, 5);    
    gpio_set(TEST_SPI1_NSS);
    if(ret>0){        
        uart_write(SERIAL2, buf, ret);
    }
}
#endif

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
    
#ifdef TEST_SPI1
    test_spi();
#endif

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


