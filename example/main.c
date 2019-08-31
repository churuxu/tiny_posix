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

#ifdef GPIO_AF12_FSMC
static SRAM_HandleTypeDef hsram1;

#define  CMD_BASE     ((uint32_t)(0x6C000000 | 0x00001FFE))
#define  DATA_BASE    ((uint32_t)(0x6C000000 | 0x00002000))

#define LCD_CMD       ( * (uint16_t *) CMD_BASE )
#define LCD_DATA      ( * (uint16_t *) DATA_BASE)

void test_fsmc(){
    int fds1 = GPIO_MULTI_FD(PORTD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
    int fds2 = GPIO_MULTI_FD(PORTE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);


    __HAL_RCC_FSMC_CLK_ENABLE();

    gpio_init(GPIO_FD(PORTF, 10), GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    gpio_init_ex(fds1, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_FSMC);
    gpio_init_ex(fds2, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_FSMC);

    gpio_init_ex(GPIO_FD(PORTG, 2), GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_FSMC);
    gpio_init_ex(GPIO_FD(PORTG, 12), GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_FSMC);

  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  //hsram1.Init.PageSize = 0;
  /* Timing */
  Timing.AddressSetupTime = 5;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 8;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

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
    int fd;
   
#ifdef GPIO_AF12_FSMC
    test_fsmc();
#endif
    //gpio_set_irq(KEY1, on_key);
    
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
    fd = open("config.xml", O_WRONLY|O_CREAT);
    if(fd>0){        
        write(fd, "<a></a>", 8);
        close(fd);
    }    

    fd = open("config.xml", O_RDONLY);
    if(fd>0){
        ret = read(fd, buf, 64);
        write(STDOUT_FILENO, buf, ret);
        close(fd);
    }

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


