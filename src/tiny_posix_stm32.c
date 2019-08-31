#include "tiny_posix.h"

#include "system_config.h"

#ifndef GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_FREQ_HIGH GPIO_SPEED_FAST
#endif

#define UART_ATTR_GET 0
#define UART_ATTR_SET 1


static int stdio_fds_[3];

void stdio_set_fd(int in, int out, int err){
    stdio_fds_[0] = in;
    stdio_fds_[1] = out;
    stdio_fds_[2] = err;
}

//stdio 输入
int stdio_read(int fd, void* buf, int len){
    if(fd == STDIN_FILENO && stdio_fds_[0]){
        return _tp_read(stdio_fds_[0], buf, len);
    }
    return -1;
}
//stdio 输出
int stdio_write(int fd, const void* buf, int len){
    if(fd == STDOUT_FILENO && stdio_fds_[1]){
        return _tp_write(stdio_fds_[1], buf, len);
    }
    if(fd == STDERR_FILENO && stdio_fds_[2]){
        return _tp_write(stdio_fds_[2], buf, len);
    }    
    return -1;
}


static void default_clock_init(){
    __HAL_RCC_PWR_CLK_ENABLE();

#ifdef __HAL_RCC_AFIO_CLK_ENABLE    
    __HAL_RCC_AFIO_CLK_ENABLE();
#endif
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



#ifdef __GNUC__
__attribute__((constructor))
#endif
void tiny_posix_init(){    
    HAL_Init();
      
    SystemClock_Config();

    default_clock_init();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    System_Config();    
}


//=============== gpio =================

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
#ifdef GPIOH
    GPIOH,
#endif
#ifdef GPIOI
    GPIOI,
#endif
};

//gpio中断回调函数
static irq_handler gpio_irqs_[16];


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
    gpio_reset(fd);
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

static USART_TypeDef* uarts_[] = {
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
    int index = UART_FD_GET_INDEX(fd);    
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
    int index = UART_FD_GET_INDEX(fd);
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
    int index = UART_FD_GET_INDEX(fd);;
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
            __HAL_RCC_UART5_CLK_ENABLE();
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

static SPI_TypeDef* spis_[] = {
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
    int index = SPI_FD_GET_INDEX(fd);   
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
    int index = SPI_FD_GET_INDEX(fd);
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
    int index = SPI_FD_GET_INDEX(fd);
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

static I2C_TypeDef* i2cs_[] = {
    I2C1,
#ifdef I2C2
    I2C2,
#endif
};

static I2C_HandleTypeDef i2c_handles_[sizeof(i2cs_)/sizeof(void*)];

//对端地址
static uint16_t i2c_peers_[sizeof(i2cs_)/sizeof(void*)]; 
//读写地址
static uint16_t i2c_memaddr_[sizeof(i2cs_)/sizeof(void*)];

static I2C_HandleTypeDef* i2c_get_handle(int fd){
    int index = I2C_FD_GET_INDEX(fd);    
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
    int index = I2C_FD_GET_INDEX(fd);   
    i2c_peers_[index] = (uint16_t)addr;
    return 0;
}

int i2c_set_offset(int fd, int off){
    int index = I2C_FD_GET_INDEX(fd);   
    i2c_memaddr_[index] = (uint16_t)off;
    return 0;
}

int i2c_set_local(int fd, int addr){
    int index = I2C_FD_GET_INDEX(fd);   
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 

    i2c->Instance             = i2cs_[index];  
    i2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2c->Init.ClockSpeed      = 400000;
    i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c->Init.DutyCycle       = I2C_DUTYCYCLE_2;
    i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    i2c->Init.OwnAddress1     = (uint8_t)(addr & 0xff);
    i2c->Init.OwnAddress2     = 0xff;

    if(HAL_OK != HAL_I2C_Init(i2c)){
        return -1;
    }
    return 0;    
}


//初始化i2c（主机）
int i2c_init(int fd, int sclpin, int sdapin, int flags, int peeraddr){
    int index = I2C_FD_GET_INDEX(fd);   
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

    i2c_set_peer(fd, peeraddr);
    return i2c_set_local(fd, flags);
}

int i2c_test(int fd){
    uint16_t peer;
    int index = I2C_FD_GET_INDEX(fd);   
    int timeout = 1000;
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 
    if(!i2c)return -1;
    peer = i2c_peers_[index];
    if(HAL_OK == HAL_I2C_IsDeviceReady(i2c, peer, 1, timeout)){
        return 0;
    }   
    return -2; 
}

//read/write 函数
int i2c_read(int fd, void* buf, int len){
    uint16_t peer;
    uint16_t mem;
    uint16_t memsz = 1;
    int index = I2C_FD_GET_INDEX(fd);   
    int recved = 0;
    int timeout ;
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 
    if(!i2c)return -1;
    timeout = 5000;
    peer = i2c_peers_[index];
    mem = i2c_memaddr_[index];
    if(HAL_OK == HAL_I2C_Mem_Read(i2c, peer, mem, memsz, (uint8_t*)buf, len, timeout)){
        recved = len;
    }    
    /*
    while(recved < len){
        if(HAL_OK != HAL_I2C_Master_Receive(i2c, peer, (uint8_t*)buf + recved, 1, timeout)){
            return recved;
        }
        recved ++;
        timeout = 32;
    } */       
    return recved;
}

int i2c_write(int fd, const void* buf, int len){
    uint16_t peer;
    uint16_t mem;
    uint16_t memsz = 1;    
    int timeout ;
    int index = I2C_FD_GET_INDEX(fd);   
    I2C_HandleTypeDef* i2c = i2c_get_handle(fd); 
    if(!i2c)return -1; 
    timeout = 5000;
    peer = i2c_peers_[index];
    mem = i2c_memaddr_[index];
    if(HAL_OK == HAL_I2C_Mem_Write(i2c, peer, mem, memsz, (uint8_t*)buf, len, timeout)){
        return len;
    }    
    /*
    if(HAL_OK == HAL_I2C_Master_Transmit(i2c, peer, (uint8_t*)buf, len, 2000)){
        return len;
    }*/
    return -1;
}

//===================== rom ===================

int rom_read(int fd, void* buf, int len){
    void* addr = ROM_FD_GET_ADDRESS(fd);
    memcpy(buf, addr, len);
    return len;
}


int rom_write(int fd, const void* buf, int buflen){
    int ret = -1;   
#if defined(FLASH_PAGE_SIZE) && defined(FLASH_TYPEERASE_PAGES)
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t error = 0;
    uint64_t* ptr = (uint64_t*)buf;
    size_t remainlen = buflen;
    HAL_StatusTypeDef state;
    uint64_t last = (uint64_t)(-1);
    
    uint32_t addr = (uint32_t)(uintptr_t)ROM_FD_GET_ADDRESS(fd);
    if(!addr || buflen>FLASH_PAGE_SIZE){
        errno = EINVAL;
        return -1;        
    }    

    HAL_FLASH_Unlock();

    pEraseInit.PageAddress = addr;
    pEraseInit.NbPages = 1;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    state = HAL_FLASHEx_Erase(&pEraseInit, &error);
    if(state != HAL_OK){        
        errno = EFAULT;
        goto finish;
    }

    while(remainlen >= 8){
        state = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, *ptr);
        if (state != HAL_OK){
            errno = EIO;
            goto finish;
        }     
        ptr += 1;
        addr += 8;
        remainlen -= 8;
    }

    if(remainlen>0 && remainlen<8){
        memcpy(&last, ptr, remainlen);        
        state = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, last);
        if (state != HAL_OK){
            errno = EIO;
            goto finish;
        }         
    }

    ret = buflen;
finish:
    HAL_FLASH_Lock();    
#endif
    return ret;    
}

//===================== lcd ===================
#define MAX_LCD_COUNT 1

static LCD_DrvTypeDef* lcd_drvs_[MAX_LCD_COUNT];
static uint16_t lcd_ids_[MAX_LCD_COUNT];
static uint8_t lcd_state_[MAX_LCD_COUNT];

int lcd_init(int fd, LCD_DrvTypeDef* driver, uint16_t lcdid){
    uint16_t readedid;
    int index = LCD_FD_GET_INDEX(fd);
    if(index >= MAX_LCD_COUNT)return -1;
    lcd_drvs_[index] = driver;
    lcd_ids_[index] = lcdid;

    driver->Init();
    readedid = driver->ReadID();
    if(lcdid && lcdid != readedid){
        return -1;
    }
    driver->DisplayOn();
    lcd_state_[index] = 1;
    return 0;
}

//turn on/off
int lcd_display(int fd, int on){
    return 0;
}

int lcd_get_width(int fd);
int lcd_get_height(int fd);

int lcd_draw(int fd, int x, int y, int w, int h, void* rgbdata);




//======================= raw disk =============================

typedef struct disk_info{    
    unsigned int total_size;
    unsigned int block_count;
    unsigned int block_size;
    unsigned int sector_count;
    unsigned int sector_size;
    unsigned int rw_pos;
    uint16_t fact; //block_size = fact * sector_size
    uint8_t inited;
    uint8_t rev;
}disk_info;

#ifndef DISK_BUFSIZE
#define DISK_BUFSIZE 4096
#endif

#define MAX_DISK_COUNT 4

static Diskio_drvTypeDef* disk_drvs_[MAX_DISK_COUNT];
static disk_info disk_info_[MAX_DISK_COUNT];




int disk_init(int fd, Diskio_drvTypeDef* driver){    
    int index = DISK_FD_GET_INDEX(fd);    
    if(index >= MAX_DISK_COUNT)return -1; 
    disk_drvs_[index] = driver;
    if(driver->disk_initialize(index)){
        return -1;
    }
    if(driver->disk_ioctl(index, GET_SECTOR_COUNT, &disk_info_[index].sector_count)){
        return -1;
    }
    if(driver->disk_ioctl(index, GET_SECTOR_SIZE, &disk_info_[index].sector_size)){
        return -1;
    }
    if(driver->disk_ioctl(index, GET_BLOCK_SIZE, &disk_info_[index].block_size)){
        return -1;
    }  
    if(disk_info_[index].block_size > DISK_BUFSIZE){
        return -1;  
    }
    disk_info_[index].fact = disk_info_[index].block_size / disk_info_[index].sector_size;
    if(!disk_info_[index].fact){
        return -1;  
    }
    disk_info_[index].inited = 1;
    disk_info_[index].total_size = disk_info_[index].sector_size * disk_info_[index].sector_count;
    disk_info_[index].block_count = disk_info_[index].total_size / disk_info_[index].block_size;
    
    return 0;  
}

int disk_lseek(int fd, int offset, int how){
    int pos;
    int index = DISK_FD_GET_INDEX(fd);
    if(index >= MAX_DISK_COUNT)return -1; 
    if(!disk_info_[index].inited)return -1;
    if(how == SEEK_END){
        pos = disk_info_[index].total_size;
    }else if(how == SEEK_CUR){
        pos = disk_info_[index].rw_pos + offset;
    }else{
        pos = offset;
    }
    if(pos<0||pos>disk_info_[index].total_size)goto error;
    if(offset % disk_info_[index].block_size)goto error;
    disk_info_[index].rw_pos = pos;
    return pos;
error: 
    disk_info_[index].rw_pos = disk_info_[index].total_size;
    return -1;
}


typedef struct disk_io_param{
    unsigned int sector_count; //要读写的块个数
    unsigned int sector_index; //要读写的块索引  
    unsigned int offset; //读到buffer 不是从0开始
    unsigned int io_len; //读到buffer 不是全部    
    unsigned int remain_len; //剩余长度
    int need_buffer; //是否需要buffer
}disk_io_param;

//计算读写扇区位置
static void disk_calc_io_param(unsigned int secsize, unsigned int secount, unsigned int pos, unsigned int len, disk_io_param* param){
    unsigned int m, n, lm, ln, total; 
    total = secsize * secount;
    if((pos + len) > total){
        len = total - pos;
    }
    if(secsize == 1){ //块大小=1，随便读写
        param->sector_count = len;
        param->sector_index = pos;
        param->offset = 0;
        param->io_len = len; 
        param->remain_len = 0;
        param->need_buffer = 0; 
        return;        
    }
    m = pos % secsize;
    n = pos / secsize;
    if(m){ //非基准点
        param->need_buffer = 1;
        param->sector_count = 1;
        param->sector_index = n;
        param->offset = m;
        param->io_len = secsize - m;
        if(param->io_len > len){
            param->remain_len = 0;
            param->io_len = len;
        }else{
            param->remain_len = len - param->io_len;
        }
    }else{ //基准点
        lm = len % secsize;
        ln = len / secsize;
        if(!lm){ //刚好倍数大小
            param->sector_count = ln;
            param->sector_index = n;
            param->offset = 0;
            param->io_len = len; 
            param->remain_len = 0;
            param->need_buffer = 0;           
        }else{ //有多余
            if(ln){ //不是最后一块
                param->sector_count = ln;
                param->sector_index = n;
                param->offset = 0;
                param->io_len = ln * secsize; 
                param->remain_len = len - param->io_len;
                param->need_buffer = 0;           
            }else{ //最后一块
                param->sector_count = 1;
                param->sector_index = n;
                param->offset = 0;
                param->io_len = len; 
                param->remain_len = 0;
                param->need_buffer = 1; 
            }
        }
    }
}


int disk_read(int fd, void* buf, int len){
    int pos;
    int readed = 0;   
    int ret;
    disk_io_param param;
    uint8_t* tempbuf = NULL;
    int index = DISK_FD_GET_INDEX(fd);
    if(index >= MAX_DISK_COUNT)return -1; 
    if(!disk_info_[index].inited)return -1;

    pos = disk_info_[index].rw_pos;
    param.remain_len = len;
    while(param.remain_len){
        disk_calc_io_param(disk_info_[index].sector_size, disk_info_[index].sector_count, pos, param.remain_len, &param);
        //printf("    read sec:%d\tnum:%d\tpos:%d\toff:%d\tlen:%d\n", param.sector_index, param.sector_count, pos, param.offset, param.io_len);
        if(param.need_buffer){ //需要临时buffer
            if(!tempbuf)tempbuf = (uint8_t*)malloc(disk_info_[index].sector_size);
            if(!tempbuf)goto error;            
        }else{
            tempbuf = (uint8_t*)buf + readed;
        }
        ret = disk_drvs_[index]->disk_read(index, tempbuf, param.sector_index, param.sector_count);
        if(ret)goto error;

        if(param.need_buffer){ //如果读到了临时buffer
            memcpy((uint8_t*)buf + readed, tempbuf + param.offset, param.io_len);         
        }

        readed += param.io_len;
        pos += param.io_len;
        disk_info_[index].rw_pos = pos;
    }

    goto finish;
error:
    readed = -1;
finish:    
    if(tempbuf)free(tempbuf);    
    return readed;
}

int disk_write(int fd, const void* buf, int len){
    int pos;
    int writed = 0;   
    int ret;
    int secindex, seccount;
    disk_io_param param;
    uint8_t* tempbuf = NULL;
    int index = DISK_FD_GET_INDEX(fd);
    if(index >= MAX_DISK_COUNT)return -1; 
    if(!disk_info_[index].inited)return -1;

    pos = disk_info_[index].rw_pos;
    param.remain_len = len;
    while(param.remain_len){
        disk_calc_io_param(disk_info_[index].block_size, disk_info_[index].block_count, pos, param.remain_len, &param);
        //printf("    read sec:%d\tnum:%d\tpos:%d\toff:%d\tlen:%d\n", param.sector_index, param.sector_count, pos, param.offset, param.io_len);
        secindex = param.sector_index * disk_info_[index].fact;
        seccount = param.sector_count * disk_info_[index].fact;
        if(param.need_buffer){ //需要临时buffer
            if(!tempbuf)tempbuf = (uint8_t*)malloc(disk_info_[index].block_size);
            if(!tempbuf)goto error; 

            //先读到临时buffer，以保留原有数据
            ret = disk_drvs_[index]->disk_read(index, tempbuf, secindex, seccount);                
            if(ret)goto error;

            memcpy(tempbuf + param.offset, buf, param.io_len);

        }else{
            tempbuf = (uint8_t*)buf + writed;
        }
        ret = disk_drvs_[index]->disk_write(index, tempbuf, secindex, seccount);
        if(ret)goto error;
        
        writed += param.io_len;
        pos += param.io_len;
        disk_info_[index].rw_pos = pos;
    }

    goto finish;
error:
    writed = -1;
finish:    
    if(tempbuf)free(tempbuf);    
    return writed;    
}


//======================== posix api =============================
typedef int (*read_func)(int fd, void* buf, int sz);
typedef int (*write_func)(int fd, const void* buf, int sz);
typedef int (*fcntl_func)(int fd, const void* buf, va_list v);

typedef struct file_ops{
    read_func rd;
    write_func wr;
    fcntl_func ctl;
}file_ops;

static file_ops file_ops_[] = {
    {stdio_read, stdio_write, NULL},
    {gpio_read,  gpio_write,  NULL},
    {uart_read,  uart_write,  NULL}, 
    {spi_read,   spi_write,   NULL}, 
    {i2c_read,   i2c_write,   NULL}, 
    {rom_read,   rom_write,  NULL}, 
};


ssize_t _tp_read(int fd, void* buf, size_t sz){
    int type = FD_GET_TYPE(fd);
    read_func func = file_ops_[type].rd;
    if(func)return func(fd, buf, sz);
    return -1;
}

ssize_t _tp_write(int fd, const void* buf, size_t sz){
    int type = FD_GET_TYPE(fd);
    write_func func = file_ops_[type].wr;
    if(func)return func(fd, buf, sz);
    return -1;
}


int _tp_open(const char* pathname, int flags, ...){
    int fd = System_Open(pathname, flags);
    if(fd>=0)return fd;
    return -1;
}

int _tp_close(int fd){
    return 0;
}


int _tp_fcntl(int fd, int cmd, ...){
    return 0;
}




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


// 





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


