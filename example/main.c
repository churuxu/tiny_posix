#include "tiny_posix.h"
#include "lcd.h"


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

#ifdef GPIO_AF12_FSMC

#define  CMD_BASE     ((uint32_t)(0x6C000000 | 0x00001FFE))
#define  DATA_BASE    ((uint32_t)(0x6C000000 | 0x00002000))

#define LCD_CMD       ( * (uint16_t *) CMD_BASE )
#define LCD_DATA      ( * (uint16_t *) DATA_BASE)

void test_fsmc(){    

     sleep(1);
 	 uint16_t id;
	
	 LCD_CMD=0xD3;	//9341读ID命令   
	 id=LCD_DATA;	 
	 id=LCD_DATA; 	//0x00
	 id=LCD_DATA;   //0x93								   
	 id<<=8;
	 id|=LCD_DATA;  //0x41  

    
    printf("aaa %04x", (int)id);
    
}

#endif


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

void test_sleep_mode(){
    //char buf[64];
    int t0 = HAL_GetTick();
    int c0 = SysTick->VAL;
    int t1, c1;
    
    sleep(2);
    printf("enter sleep");
    //printf(SERIAL2, buf, len);
    gpio_set(leds_[0]);
    HAL_SuspendTick();    
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);     
    HAL_ResumeTick();
    t1 = HAL_GetTick();
    c1 = SysTick->VAL;
    printf("leave sleep %d~%d %d~%d",t0,t1,c0,c1);
    //uart_write(SERIAL2, buf, len);
    
}


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

/*
void test_lcd(){
    gpio_set(GPIO_FD(PORTF,10));
 	LCD_Init();
	BRUSH_COLOR=RED;	
	LCD_DisplayString(10,10,24,"Illuminati STM32F4");	
	LCD_DisplayString(20,40,16,"Author:Clever");
	LCD_DisplayString(30,80,24,"4.TFTLCD TEST");
}*/

int main(){
    char buf[64];    
    int ret;
    int fd,i;
   
    //test_lcd();


    leds_[0] = open("/leds/led0", O_WRONLY);

    fd = open("/keys/key0", O_RDONLY);
    gpio_set_irq(fd, on_key);
    
#ifdef TEST_SPI1
    //test_spi();
#endif
#ifdef TEST_I2C1
    //test_i2c();
#endif
    //test_loop_led();
    //test_sleep_mode();
    //FILE* f = fopen("a","1");
    //printf("a");
    //fwrite("aaa",1,2,f);
    //fclose(f);
    /*
    fd = open("config.xml", O_WRONLY|O_CREAT);
    if(fd>0){        
        write(fd, "<a></a>", 8);
        close(fd);
    }    */
    fd = open("config.xml", O_RDONLY);
    if(fd>0){
        ret = read(fd, buf, 64);
        if(ret>0)write(STDOUT_FILENO, buf, ret);
        close(fd);
    }
    i = 0;
    while(1){
        i ++;
        gpio_toggle(leds_[0]);
        printf("hello world %d\n", i);
        sleep(1);
    }
    
    while(1){
        //loop_leds();
        gpio_toggle(leds_[0]);
        printf("hello world\n");
        sleep(1);
        /*
        ret = read(STDIN_FILENO, buf, 64);
        if(ret>0){
            write(STDOUT_FILENO, buf, ret);
        }*/
    }

    return 0;
}


