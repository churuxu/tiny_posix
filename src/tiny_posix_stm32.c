
#include "tiny_posix_stm32.h"
#include "tiny_posix.h"

#ifndef GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_FREQ_HIGH GPIO_SPEED_FAST
#endif

#define UART_ATTR_GET 0
#define UART_ATTR_SET 1

//=============== 全局对象 =================

GPIO_TypeDef* gpio_ports_[] = {
    GPIOA,
    GPIOB,
#ifdef GPIOC    
    GPIOC,
#endif    
#ifdef GPIOD
    GPIOD,
#endif
#ifdef GPIOE
    GPIOE,
#endif
#ifdef GPIOF
    GPIOF,
#endif
#ifdef GPIOG
    GPIOG,
#endif
};

//gpio中断回调函数
static irq_handler gpio_irqs_[16];



extern void SystemClock_Config();
extern void HAL_Config();

static void default_clock_init(){
    __HAL_RCC_PWR_CLK_ENABLE();
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


void tiny_posix_init(){    
    HAL_Init();

    default_clock_init();
    
    SystemClock_Config();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    HAL_Config();    
}

int gpio_init_ex(int fd, int mode, int pull, int af){
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_FD_GET_PIN(fd);
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#ifdef IS_GPIO_AF 
    GPIO_InitStruct.Alternate = af;
#endif
    HAL_GPIO_Init(GPIO_FD_GET_PORT(fd), &GPIO_InitStruct);   
    //gpio_reset(fd);
    return 0;    
}


int gpio_init(int fd, int mode, int pull){    
    return gpio_init_ex(fd, mode, pull, 0);    
}

void gpio_set_irq(int fd, irq_handler func){
    IRQn_Type IRQnb = EXTI0_IRQn;
    uint32_t priority = 0;
    uint16_t pin = GPIO_FD_GET_PIN(fd);
    int index = 0;
    switch(pin) {
        case GPIO_PIN_0:  index = 0;  IRQnb = EXTI0_IRQn;	break;
        case GPIO_PIN_1:  index = 1;  IRQnb = EXTI1_IRQn;	break;
        case GPIO_PIN_2:  index = 2;  IRQnb = EXTI2_IRQn;	break;
        case GPIO_PIN_3:  index = 3;  IRQnb = EXTI3_IRQn;	break;
        case GPIO_PIN_4:  index = 4;  IRQnb = EXTI4_IRQn;	break;
        case GPIO_PIN_5:  index = 5;  IRQnb = EXTI9_5_IRQn; break;
        case GPIO_PIN_6:  index = 6;  IRQnb = EXTI9_5_IRQn;	break;
        case GPIO_PIN_7:  index = 7;  IRQnb = EXTI9_5_IRQn;	break;
        case GPIO_PIN_8:  index = 8;  IRQnb = EXTI9_5_IRQn;	break;
        case GPIO_PIN_9:  index = 9;  IRQnb = EXTI9_5_IRQn; break;
        case GPIO_PIN_10: index = 10; IRQnb = EXTI15_10_IRQn; break;
        case GPIO_PIN_11: index = 11; IRQnb = EXTI15_10_IRQn; break;
        case GPIO_PIN_12: index = 12; IRQnb = EXTI15_10_IRQn; break;
        case GPIO_PIN_13: index = 13; IRQnb = EXTI15_10_IRQn; break;
        case GPIO_PIN_14: index = 14; IRQnb = EXTI15_10_IRQn; break;
        case GPIO_PIN_15: index = 15; IRQnb = EXTI15_10_IRQn; break;
        default:return;
    }
    gpio_irqs_[index] = func;

    HAL_NVIC_SetPriority(IRQnb, priority, 0);
    HAL_NVIC_EnableIRQ(IRQnb);    
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



//======================== uart ========================

USART_TypeDef* uarts_[] = {
    USART1,
    USART2,
#ifdef USART3   
    USART3,
#endif     
#ifdef UART4
    UART4,
#endif 
#ifdef UART5
    UART5,
#endif     
};


UART_HandleTypeDef uart_handles_[sizeof(uarts_)/sizeof(void*)];



static UART_HandleTypeDef* uart_get_handle(int fd){
    int index = (fd>>8);    
    return &(uart_handles_[index]);
}

static int uart_get_gpio_af(int index){
#ifdef GPIO_AF7_USART1
    if(index == 0)return GPIO_AF7_USART1;
#endif
#ifdef GPIO_AF7_USART2
    if(index == 1)return GPIO_AF7_USART2;
#endif
#ifdef GPIO_AF7_USART3
    if(index == 2)return GPIO_AF7_USART3;
#endif
#ifdef GPIO_AF8_UART4
    if(index == 3)return GPIO_AF8_UART4;
#endif
#ifdef GPIO_AF8_UART5
    if(index == 4)return GPIO_AF8_UART5;
#endif
    return 0;
}


static int uart_get_flags(int fd){
    int baud;
    int stopbits;
    int parity;
     
    UART_HandleTypeDef* uart = uart_get_handle(fd);
    if(!uart)return 0;

    baud = (uart->Init.BaudRate / 100);
    if(uart->Init.Parity == UART_PARITY_ODD){
        parity = PARENB | PARODD;
    }else if(uart->Init.Parity == UART_PARITY_EVEN){
        parity = PARENB;
    }else{
        parity = 0;
    }
    stopbits = (uart->Init.StopBits == UART_STOPBITS_2)?CSTOPB:0;

    return baud|CS8|parity|stopbits;
}

int uart_set_flags(int fd, int flags){
    int baud;
    int stopbits;
    int parity;
    int wordlen;
    int index = (fd>>8);
    UART_HandleTypeDef* uart = uart_get_handle(fd);   
    if(!uart)return -1;

    baud = ((flags & 0xffff) * 100);
    stopbits = (flags & CSTOPB)? UART_STOPBITS_2 : UART_STOPBITS_1;
    parity = (flags & PARENB)?((flags & PARODD)?UART_PARITY_ODD:UART_PARITY_EVEN):UART_PARITY_NONE;
    wordlen = (flags & PARENB)? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    uart->Instance = uarts_[index];
    uart->Init.BaudRate = baud;
    uart->Init.WordLength = wordlen;
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




int uart_init(int fd, int tx, int rx, int flags){    
    int index = (fd>>8);
    int af = uart_get_gpio_af(index);

    switch(index){
        case 0: 
            __HAL_RCC_USART1_CLK_ENABLE();
            HAL_NVIC_EnableIRQ(USART1_IRQn);
            HAL_NVIC_SetPriority(USART1_IRQn,0,0);
            break;
        case 1: 
            __HAL_RCC_USART2_CLK_ENABLE();
            HAL_NVIC_EnableIRQ(USART2_IRQn);
            HAL_NVIC_SetPriority(USART2_IRQn,0,0);
            break;
#ifdef USART3        
        case 2: 
            __HAL_RCC_USART3_CLK_ENABLE();
            HAL_NVIC_EnableIRQ(USART3_IRQn);
            HAL_NVIC_SetPriority(USART3_IRQn,0,0);
            break;
#endif
#ifdef UART4        
        case 3: 
            __HAL_RCC_UART4_CLK_ENABLE();
            HAL_NVIC_EnableIRQ(UART4_IRQn);
            HAL_NVIC_SetPriority(UART4_IRQn,0,0);
            break;
#endif
#ifdef UART5 
        case 4: 
            __HAL_RCC_USART5_CLK_ENABLE();
            HAL_NVIC_EnableIRQ(UART5_IRQn);
            HAL_NVIC_SetPriority(UART5_IRQn,0,0);
            break;
#endif        
        default:return -1;
    }
    
    gpio_init_ex(tx, GPIO_MODE_AF_PP, GPIO_NOPULL, af);
    gpio_init_ex(rx, GPIO_MODE_INPUT, GPIO_NOPULL, af);
    
    return uart_set_flags(fd, flags);    
}

int uart_config(int fd, int key, void* value){
    //struct termios* attr;
    switch(key){
        case UART_ATTR_SET:

        case UART_ATTR_GET:

        default:
            return 0;
    }
    return 0;
}




int uart_read(int fd, void* buf, int len){
    int recved = 0;
    int timeout ;
    UART_HandleTypeDef* uart = uart_get_handle(fd);
    if(!uart)return -1;
    timeout = 2000;
    while(recved < len){
        if(HAL_OK != HAL_UART_Receive(uart, (uint8_t*)buf + recved, 1, timeout)){
            return recved;
        }
        recved ++;
        timeout = 32;
    }        
    return recved;
}
int uart_write(int fd, const void* buf, int len){
    UART_HandleTypeDef* uart = uart_get_handle(fd); 
    if(!uart)return -1; 
    if(HAL_OK == HAL_UART_Transmit(uart, (uint8_t*)buf, len, 2000)){
        return len;
    }
    return -1;
}


int _tp_cfsetispeed(struct termios* attr, speed_t t){
    attr->c_cflag &= 0xffff0000;
    attr->c_cflag |= t;
    return 0;
}
int _tp_cfsetospeed(struct termios* attr, speed_t t){
    attr->c_cflag &= 0xffff0000;
    attr->c_cflag |= t;
    return 0;    
}
int _tp_tcgetattr(int fd, struct termios* attr){
    int flag = uart_get_flags(fd);
    attr->c_cflag = flag;
    return (flag == 0);
}
int _tp_tcsetattr(int fd, int opt, const struct termios* attr){
    return uart_set_flags(fd, attr->c_cflag);
}
//=====================================================


//======================== spi ========================

SPI_TypeDef* spis_[] = {
    SPI1,
#ifdef SPI2
    SPI2,
#endif
#ifdef SPI3
    SPI3,
#endif
};


SPI_HandleTypeDef spi_handles_[sizeof(spis_)/sizeof(void*)];

static SPI_HandleTypeDef* spi_get_handle(int fd){
    int index = (fd>>8);    
    return &(spi_handles_[index]);
}


static int spi_get_gpio_af(int index){
#ifdef GPIO_AF5_SPI1
    if(index == 0)return GPIO_AF5_SPI1;
#endif
#ifdef GPIO_AF5_SPI2
    if(index == 1)return GPIO_AF5_SPI2;
#endif
#ifdef GPIO_AF5_SPI3
    if(index == 2)return GPIO_AF5_SPI3;
#endif
    return 0;
}

static int spi_set_flags(int fd, int flags){
    int index = (fd>>8);
    SPI_HandleTypeDef* spi = spi_get_handle(fd); 

    spi->Instance = spis_[index];
    spi->Init.Mode = SPI_MODE_MASTER;
    spi->Init.Direction = SPI_DIRECTION_2LINES;
    spi->Init.DataSize = SPI_DATASIZE_8BIT;
    spi->Init.CLKPolarity = SPI_POLARITY_LOW;
    spi->Init.CLKPhase = SPI_PHASE_1EDGE;
    spi->Init.NSS = SPI_NSS_SOFT;
    spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi->Init.TIMode = SPI_TIMODE_DISABLE;
    spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->Init.CRCPolynomial = 10;

    if(HAL_OK != HAL_SPI_Init(spi)){
        return -1;
    }
    return 0;    
}

int spi_init(int fd, int clkpin, int misopin, int mosipin){
    int index = (fd>>8);
    int af = spi_get_gpio_af(index); 

    //clk enable
    switch(index){
        case 0: __HAL_RCC_SPI1_CLK_ENABLE(); break;
#ifdef SPI2          
        case 1: __HAL_RCC_SPI2_CLK_ENABLE(); break;
#endif
#ifdef SPI3       
        case 2: __HAL_RCC_SPI3_CLK_ENABLE(); break;
#endif  
        default:return -1;
    }

    gpio_init_ex(clkpin,  GPIO_MODE_AF_PP, GPIO_NOPULL, af);
    gpio_init_ex(misopin, GPIO_MODE_INPUT, GPIO_NOPULL, af);
    gpio_init_ex(mosipin, GPIO_MODE_AF_PP, GPIO_NOPULL, af);        

    return spi_set_flags(fd, 0);
}


int spi_io(int fd, const void* out, void* in, int len){
    SPI_HandleTypeDef* spi = spi_get_handle(fd); 
    if(!spi)return -1;
    if(HAL_OK == HAL_SPI_TransmitReceive(spi, (uint8_t*)out, (uint8_t*)in, len, 640)){
        return len;
    }
    return -1;   
}

int spi_read(int fd, void* buf, int len){
    int recved = 0;
    int timeout ;
    SPI_HandleTypeDef* spi = spi_get_handle(fd); 
    if(!spi)return -1;
    timeout = 320;
    while(recved < len){
        if(HAL_OK != HAL_SPI_Receive(spi, (uint8_t*)buf + recved, 1, timeout)){
            return recved;
        }
        recved ++;
        timeout = 16;
    }        
    return recved;    
}

int spi_write(int fd, const void* buf, int len){
    SPI_HandleTypeDef* spi = spi_get_handle(fd); 
    if(!spi)return -1; 
    if(HAL_OK == HAL_SPI_Transmit(spi, (uint8_t*)buf, len, 320)){
        return len;
    }
    return -1;
}

//======================== I2C =============================

I2C_TypeDef* i2cs_[] = {
    I2C1,
#ifdef I2C2
    I2C2,
#endif
};

I2C_HandleTypeDef i2c_handles_[sizeof(i2cs_)/sizeof(void*)];

//对端地址
uint16_t i2c_peers_[sizeof(i2cs_)/sizeof(void*)]; 


static I2C_HandleTypeDef* i2c_get_handle(int fd){
    int index = (fd>>8);    
    return &(i2c_handles_[index]);
}


static int i2c_get_gpio_af(int index){
#ifdef GPIO_AF4_I2C1
    if(index == 0)return GPIO_AF4_I2C1;
#endif
#ifdef GPIO_AF4_I2C2
    if(index == 1)return GPIO_AF4_I2C2;
#endif
    return 0;
}

int i2c_set_peer(int fd, int addr){
    int index = (fd>>8);
    i2c_peers_[index] = (uint16_t)addr;
    return 0;
}

int i2c_set_local(int fd, int addr){
    int index = (fd>>8);
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 

    i2c->Instance             = i2cs_[index];  
    i2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2c->Init.ClockSpeed      = 400000;
    i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c->Init.DutyCycle       = I2C_DUTYCYCLE_2;
    i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    i2c->Init.OwnAddress1     = (uint8_t)(addr & 0xff);
    i2c->Init.OwnAddress2     = (uint8_t)((addr & 0xff00)>>8);

    if(HAL_OK != HAL_I2C_Init(i2c)){
        return -1;
    }
    return 0;    
}


//初始化i2c（主机）
int i2c_init(int fd, int sclpin, int sdapin, int flags){
    int index = (fd>>8);
    int af = i2c_get_gpio_af(index); 

    //clk enable
    switch(index){
        case 0: __HAL_RCC_I2C1_CLK_ENABLE(); break;
#ifdef I2C2
        case 1: __HAL_RCC_I2C2_CLK_ENABLE(); break;
#endif
        default:return -1;
    }

    gpio_init_ex(sclpin, GPIO_MODE_AF_OD, GPIO_PULLUP, af);
    gpio_init_ex(sdapin, GPIO_MODE_AF_OD, GPIO_PULLUP, af);

    return i2c_set_local(fd, flags);
}

//read/write 函数
int i2c_read(int fd, void* buf, int len){
    uint16_t peer;
    int index = (fd>>8);
    int recved = 0;
    int timeout ;
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 
    if(!i2c)return -1;
    timeout = 2000;
    peer = i2c_peers_[index];
    while(recved < len){
        if(HAL_OK != HAL_I2C_Master_Receive(i2c, peer, (uint8_t*)buf + recved, 1, timeout)){
            return recved;
        }
        recved ++;
        timeout = 32;
    }        
    return recved;
}

int i2c_write(int fd, const void* buf, int len){
    uint16_t peer;
    int index = (fd>>8);
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 
    if(!i2c)return -1; 
    peer = i2c_peers_[index];
    if(HAL_OK == HAL_I2C_Master_Transmit(i2c, peer, (uint8_t*)buf, len, 2000)){
        return len;
    }
    return -1;
}







//======================== SYS =============================
unsigned int _tp_sleep(unsigned int seconds){
    HAL_Delay(seconds * 1000);
    return 0;
}
unsigned int _tp_usleep(unsigned int micro_seconds){
    HAL_Delay(micro_seconds / 1000);
    return 0;
}

_tp_clock_t _tp_clock(){
    return HAL_GetTick();
}








#define HANDLE_GPIO_IRQ(pin, index) \
    if(__HAL_GPIO_EXTI_GET_IT(pin) != RESET){\
        __HAL_GPIO_EXTI_CLEAR_IT(pin);\
        if(gpio_irqs_[index])gpio_irqs_[index]();\
    }

//============= 各系统中断 ===============
void SysTick_Handler(){ 
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler(); 
}
void EXTI0_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_0, 0);
}
void EXTI1_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_1, 1);
}
void EXTI2_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_2, 2);
}
void EXTI3_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_3, 3);
}
void EXTI4_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_4, 4);
}
void EXTI9_5_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_5, 5);
    HANDLE_GPIO_IRQ(GPIO_PIN_6, 6);
    HANDLE_GPIO_IRQ(GPIO_PIN_7, 7);
    HANDLE_GPIO_IRQ(GPIO_PIN_8, 8);
    HANDLE_GPIO_IRQ(GPIO_PIN_9, 9);
}
void EXTI15_10_IRQHandler(){
    HANDLE_GPIO_IRQ(GPIO_PIN_10, 10);
    HANDLE_GPIO_IRQ(GPIO_PIN_11, 11);
    HANDLE_GPIO_IRQ(GPIO_PIN_12, 12);
    HANDLE_GPIO_IRQ(GPIO_PIN_13, 13);
    HANDLE_GPIO_IRQ(GPIO_PIN_14, 14);
    HANDLE_GPIO_IRQ(GPIO_PIN_15, 15);
}


