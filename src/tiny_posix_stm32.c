
#include "tiny_posix_stm32.h"
#include "tiny_posix_driver.h"
#include "tiny_posix.h"

#ifndef GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_FREQ_HIGH GPIO_SPEED_HIGH
#endif

#define UART_ATTR_GET 0
#define UART_ATTR_SET 1

//=============== 全局对象 =================

GPIO_TypeDef* gpio_ports_[] = {
    GPIOA,
    GPIOB,
    GPIOC
};

USART_TypeDef* uarts_[] = {
    USART1,
    USART2,
    USART3,
};

UART_HandleTypeDef uart_handles_[8];

/*
clock
HSI 高速内部时钟，8MHz
LSI 低速内部时钟，40kHz
HSE 高速外部时钟，4MHz~16MHz
LSE 低速外部时钟，32.768kHz

PLL HSI/2、HSE或者HSE/2

*/

extern void SystemClock_Config();
extern void HAL_Config();

static void gpio_clock_init(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();   
    __HAL_RCC_GPIOC_CLK_ENABLE();
#ifdef GPIOD
    __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
    __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
    __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
    __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
}


int tiny_posix_init(){
    int ret = 0;
    HAL_Init();

    gpio_clock_init();
    
    SystemClock_Config();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    HAL_Config();

    return ret;
}


int gpio_init(int fd, int flags){
    int pull, mod;
    int flagm, flagp;
    flagm = ((flags >> 8) & 0xff);
    flagp = ((flags ) & 0xff);

    mod = GPIO_MODE_OUTPUT_PP;
    if(flagm == GPIO_FLAGS_INPUT){
        mod = GPIO_MODE_INPUT;
    }    
    pull = GPIO_NOPULL;
    if(flagp == GPIO_FLAGS_PULL_UP){
        pull = GPIO_PULLUP;
    }else if(flagp == GPIO_FLAGS_PULL_DOWN){
        pull = GPIO_PULLDOWN;
    } 

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_FD_GET_PIN(fd);
    GPIO_InitStruct.Mode = mod;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_FD_GET_PORT(fd), &GPIO_InitStruct);   

    return 0; 
}


int gpio_config(int fd, int key, void* value){
    return 0;
}
int gpio_read(int fd, void* buf, int len){
    uint8_t* pbuf = (uint8_t*)buf;
    int val = gpio_status(fd);
    *pbuf = val?1:0;
    return 1;
}
int gpio_write(int fd, const void* buf, int len){
    uint8_t* pbuf = (uint8_t*)buf;
    if(*pbuf){
        gpio_set(fd);
    }else{
        gpio_reset(fd);
    }    
    return 1;
}

static UART_HandleTypeDef* uart_get_handle(int fd){
    int index = (fd>>8);    
    return &(uart_handles_[index]);
}

static int uart_get_attr(int fd, struct termios* attr){


    return 0;
}
static int uart_set_attr(int fd, const struct termios* attr){
    int baud;
    int stopbits;
    int parity;
    int index = (fd>>8);
    UART_HandleTypeDef* uart = uart_get_handle(fd);   
    
    baud = ((attr->c_cflag & 0xffff) * 100);
    stopbits = (attr->c_cflag & CSTOPB)? UART_STOPBITS_2 : UART_STOPBITS_1;
    parity = (attr->c_cflag & PARENB)?((attr->c_cflag & PARODD)?UART_PARITY_ODD:UART_PARITY_EVEN):UART_PARITY_NONE;
     
    uart->Instance = uarts_[index];
    uart->Init.BaudRate = baud;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = stopbits;
    uart->Init.Parity = parity;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_OK != HAL_UART_Init(uart)){
        return -1;
    }
    return 0;
}

int uart_init(int fd, int flags){
    struct termios attr;
    attr.c_cflag = flags;
    return uart_set_attr(fd, &attr);    
}

int uart_config(int fd, int key, void* value){
    struct termios* attr;
    switch(key){
        case UART_ATTR_SET:

        case UART_ATTR_GET:

        default:
            return 0;
    }
    return 0;
}
int uart_read(int fd, void* buf, int len){
    return 0;
}
int uart_write(int fd, const void* buf, int len){
    UART_HandleTypeDef* uart = uart_get_handle(fd);  
    if(HAL_OK == HAL_UART_Transmit(uart, (uint8_t*)buf, len, 3000)){
        return len;
    }
    return -1;
}


int _tp_cfsetispeed(struct termios* attr, speed_t t){
    attr->c_cflag &= 0x0000;
    attr->c_cflag |= t;
    return 0;
}
int _tp_cfsetospeed(struct termios* attr, speed_t t){
    attr->c_cflag &= 0x0000;
    attr->c_cflag |= t;
    return 0;    
}
int _tp_tcgetattr(int fd, struct termios* attr){
    return uart_get_attr(fd, attr);
}
int _tp_tcsetattr(int fd, int opt, const struct termios* attr){
    return uart_set_attr(fd, attr);
}

unsigned int _tp_sleep(unsigned int seconds){
    HAL_Delay(seconds * 1000);
    return 0;
}
unsigned int _tp_usleep(unsigned int micro_seconds){
    HAL_Delay(micro_seconds / 1000);
    return 0;
}




void SysTick_Handler(void){ 
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler(); 
}

