
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
}

static int SystemClock_Config(){
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        return -1;
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        return -1;
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        return -1;
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    return 0;
}


int tiny_posix_init(){
    int ret = 0;
    HAL_Init();
    ret += SystemClock_Config();
    gpio_clock_init();
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

