#include "tiny_posix.h"

static int leds_[8];
static int ledcount_;

void loop_leds(){
    static int on = 1;
    static int cur = 0; 
    if(!ledcount_) return ;  
    if(on){
        gpio_set(leds_[cur]);
    }else{
        gpio_reset(leds_[cur]);
    }
    cur ++;
    if(cur == ledcount_){
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

#ifdef TEST_I2C1
void test_i2c(){
    int ret;
    char buf[16];
    const char* msg = "hi!";

    sleep(1);

    if(i2c_test(TEST_I2C1)){
        uart_write(SERIAL2, "not i2c", 6);
        return;
    }

    ret = i2c_write(TEST_I2C1, msg, 1);
    sleep(1);
    ret = i2c_read(TEST_I2C1, buf, 1);
    if(ret>0){
        uart_write(SERIAL2, buf, ret);
    }
}

#endif
/*
void test_sleep_mode(){
    char buf[64];
    int t0 = HAL_GetTick();
    int c0 = SysTick->VAL;
    int t1, c1, len;
    
    sleep(2);
    len = sprintf(buf, "enter sleep");
    //printf(SERIAL2, buf, len);

    HAL_SuspendTick();    
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);     
    HAL_ResumeTick();
    t1 = HAL_GetTick();
    c1 = SysTick->VAL;
    len = sprintf(buf, "leave sleep %d~%d %d~%d",t0,t1,c0,c1);
    //uart_write(SERIAL2, buf, len);
}
*/

void on_key(){
    //gpio_set(LED1);
}

void test_loop_led(){
    char buf[64];
    int i, fd;
    //open leds
    for(i=0;i<8;i++){
        snprintf(buf, 64, "/leds/led%d",i);
        fd = open(buf, O_WRONLY);
        if(fd>=0){
            leds_[i] = fd;
            ledcount_ ++;
        }
    }

    while(1){
        loop_leds();
        sleep(1);
    }
}

int main(){
    char buf[64];    
    int ret;
   

    //gpio_set_irq(KEY1, on_key);
    
#ifdef TEST_SPI1
    //test_spi();
#endif
#ifdef TEST_I2C1
    //test_i2c();
#endif
    test_loop_led();
    //test_sleep_mode();
    //FILE* f = fopen("a","1");
    //printf("a");
    //fwrite("aaa",1,2,f);
    //fclose(f);
    while(1){
        //loop_leds();

        printf("hello world\n");

        ret = read(STDIN_FILENO, buf, 64);
        if(ret>0){
            write(STDOUT_FILENO, buf, ret);
        }
    }

    return 0;
}


