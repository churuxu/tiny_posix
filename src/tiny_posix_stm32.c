#include "tiny_posix.h"

#ifdef STM32

#include "system_config.h"

#include <stdarg.h>

#ifndef GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_FREQ_HIGH GPIO_SPEED_FAST
#endif

#define FCNTL_CMD_BASE 0x0C00
#define UART_ATTR_GET (FCNTL_CMD_BASE|1)
#define UART_ATTR_SET (FCNTL_CMD_BASE|2)


static int stdio_fds_[3];
static read_func stdin_func_;
static write_func stdout_func_;
static write_func stderr_func_;

void stdio_set_fd(int in, int out, int err){
    stdio_fds_[0] = in;
    stdio_fds_[1] = out;
    stdio_fds_[2] = err;
}

void stdio_set_func(read_func in, write_func out, write_func err){
    stdin_func_ = in;
    stdout_func_ = out;
    stderr_func_ = err;
}

//stdio 输入
int stdio_read(int fd, void* buf, int len){
    if(stdin_func_){
        return stdin_func_(fd, buf, len);
    }
    if(fd == STDIN_FILENO && stdio_fds_[0]){
        return _tp_read(stdio_fds_[0], buf, len);
    }
    return -1;
}

//stdio 输出
int stdio_write(int fd, const void* buf, int len){
    if(fd == STDOUT_FILENO){
        if(stdout_func_){
            return stdout_func_(fd, buf, len);
        }
        if(stdio_fds_[1]){
            return _tp_write(stdio_fds_[1], buf, len);
        }
    }
    if(fd == STDERR_FILENO){
        if(stderr_func_){
            return stderr_func_(fd, buf, len);
        }
        if(stdio_fds_[1]){
            return _tp_write(stdio_fds_[2], buf, len);
        }
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

//===============  fifo ================

typedef struct fifo{
	uint8_t* begin;  //buffer开头
	uint8_t* end;	//buffer结尾
	uint8_t* posread;  //读位置
	uint8_t* poswrite;	 //写位置	
}fifo;


static void fifo_init(fifo* result, void* buf, size_t sz){
	result->begin = (uint8_t*)buf;
	result->end = (uint8_t*)buf + sz;	
	result->poswrite = result->begin;
	result->posread = result->begin;
}
/*
static int fifo_is_full(fifo* f){
    uint8_t* nextpos = f->poswrite; 
    if(nextpos == f->end){
        nextpos = f->begin;
    }else{
        nextpos ++;
    }
    return nextpos == f->posread;
}*/

static int fifo_is_empty(fifo* f){
    return f->posread == f->poswrite;
}
static int fifo_push(fifo* f, uint8_t ch){
    uint8_t* posread;
    uint8_t* nextpos; 
    posread = f->posread; 
    nextpos = f->poswrite;
    if(nextpos == f->end){
        nextpos = f->begin;
    }else{
        nextpos ++;
    }
    if(nextpos == posread){ //下一个字节 = 读位置，说明已满
        return -1;
    }
    *f->poswrite = ch;
    f->poswrite = nextpos;
    return 0;
}

static int fifo_pop(fifo* f, uint8_t* pch){
    uint8_t* poswrite; 
    poswrite = f->poswrite;
    if(f->posread == poswrite){ //当前位置 = 写位置，说明已空
        return -1;
    }        
    *pch = *f->posread;
    if(f->posread == f->end){
        f->posread = f->begin;
    }else{
        f->posread ++;
    }  
    return 0;
}

static int fifo_write(fifo* f, const uint8_t* buf, int buflen){		
    uint8_t* posread;     
    uint8_t* nextpos; 
    int i;       
	if(!buf || buflen<=0)return 0;	
    posread = f->posread;    
    for(i=0; i<buflen; i++){
        nextpos = f->poswrite;
        if(nextpos == f->end){
            nextpos = f->begin;
        }else{
            nextpos ++;
        }        
        if(nextpos == posread){ //下一个字节 = 读位置，说明已满
            break;
        }
        *f->poswrite = *buf;
        f->poswrite = nextpos;
        buf ++;        
    }
	return i;
}


static int fifo_read(fifo* f, uint8_t* buf, int buflen){		
    uint8_t* poswrite;    
    int i;       
	if(!buf || buflen<=0)return 0;	
    poswrite = f->poswrite;    
    for(i=0; i<buflen; i++){
        if(f->posread == poswrite){ //当前位置 = 写位置，说明已空
            break;
        }        
        *buf = *f->posread;
        if(f->posread == f->end){
            f->posread = f->begin;
        }else{
            f->posread ++;
        } 
        buf ++;      
    }
	return i;
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
//gpio中断的pin
static uint16_t gpio_irq_pins_;

//中断处理函数，中断中调用HAL库，HAL库回调此函数
void HAL_GPIO_EXTI_Callback(uint16_t pin){
    uint8_t index = 0;
    gpio_irq_pins_ |= pin;
    if( pin > 0 ) {
        while( pin != 0x01 ){
            pin = pin >> 1;
            index++;
        }
    }
    if(gpio_irqs_[index]){
        gpio_irqs_[index]();
    }
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
    gpio_reset(fd);
    switch(mode){
        case GPIO_MODE_IT_RISING:
        case GPIO_MODE_IT_FALLING:
        case GPIO_MODE_IT_RISING_FALLING:
            gpio_set_irq(fd, NULL);
        default:
            break;
    }
    
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



int gpio_fcntl(int fd, int key, void* value){
    return -1;
}

int gpio_poll(int fd, int event){
    int ev = 0;
    uint16_t pin;
    if(event & POLLOUT){
        ev |= POLLOUT;
    } 
    if((event & POLLPRI) || (event & POLLIN)){
        pin = GPIO_FD_GET_PIN(fd);
        if(gpio_irq_pins_ & pin){
            gpio_irq_pins_ &= ~pin;
            ev |= event;            
        }
    }
    return ev;
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


#define UART_FIFO_BUFSIZE 512

typedef struct UART_Object{
    UART_HandleTypeDef handle;
    int flags; //fd选项
    fifo txfifo; //发送队列
    fifo rxfifo; //接收队列
    uint8_t* buf;    
    uint8_t rxch;
    uint8_t txch;
}UART_Object;

static UART_Object uart_objs_[sizeof(uarts_)/sizeof(void*)];


//发送中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* handle){
    UART_Object* obj = (UART_Object*)handle;
    int ret = fifo_pop(&obj->txfifo, &obj->txch);
    if(ret == 0){ //有数据要发
        HAL_UART_Transmit_IT(handle, &obj->txch, 1);
    }
}

//接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle){
    UART_Object* obj = (UART_Object*)handle;
    fifo_push(&obj->rxfifo, obj->rxch);
    HAL_UART_Receive_IT(handle, &obj->rxch, 1);
}

//出错时恢复接收
void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle ){
    UART_Object* obj = (UART_Object*)handle;
    HAL_UART_Receive_IT(handle, &obj->rxch, 1);
}


static UART_HandleTypeDef* uart_get_handle(int fd){
    int index = UART_FD_GET_INDEX(fd);    
    return &(uart_objs_[index].handle);
}

static UART_Object* uart_get_object(int fd){
    int index = UART_FD_GET_INDEX(fd);    
    return &(uart_objs_[index]);
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


static int uart_get_attr(int fd){
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

int uart_set_attr(int fd, int flags){
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
#ifdef GPIO_AF7_USART1 //have af
    gpio_init_ex(rx, GPIO_MODE_AF_PP, GPIO_NOPULL, af);
#else
    gpio_init_ex(rx, GPIO_MODE_INPUT, GPIO_NOPULL, af);
#endif 

    return uart_set_attr(fd, flags);    
}


static int uart_set_flags(int fd, int flags){
    UART_Object* obj = (UART_Object*)uart_get_handle(fd);    
    if(flags & O_NONBLOCK){ //非阻塞，分配内存
        if(!obj->buf){
            obj->buf = (uint8_t*)malloc(UART_FIFO_BUFSIZE*2);
            if(!obj->buf){
                errno = ENOMEM;
                return -1;
            }
        }
        fifo_init(&obj->txfifo, obj->buf, UART_FIFO_BUFSIZE);
        fifo_init(&obj->rxfifo, obj->buf + UART_FIFO_BUFSIZE, UART_FIFO_BUFSIZE);

        //开始异步接收
        HAL_UART_Receive_IT(&obj->handle, &obj->rxch, 1);
    }
    obj->flags = flags;
    return 0;
}

int uart_fcntl(int fd, int key, void* value){
    int iv;
    struct termios* attr;
    UART_Object* obj = (UART_Object*)uart_get_handle(fd);
    if(!obj)return -1;
    switch(key){
        case F_GETFL: //获取flags
            *(int*)value = obj->flags;
            return 0;
        case F_SETFL: //设置flags
            iv = (int)(uintptr_t)value;
            return uart_set_flags(fd, iv);
        case UART_ATTR_GET: //获取termios属性
            attr = (struct termios*)value;
            attr->c_cflag = uart_get_attr(fd);
            return 0;
        case UART_ATTR_SET: //设置termios属性
            attr = (struct termios*)value;            
            return uart_set_attr(fd, attr->c_cflag);
        default:
            break;
    }
    return -1;
}


int uart_read(int fd, void* buf, int len){
    int recved = 0;
    int timeout ;
    UART_Object* uart = uart_get_object(fd);
    if(!uart)return -1;
    //非阻塞读取
    if(uart->flags & O_NONBLOCK){
        recved = fifo_read(&uart->rxfifo, buf, len);
        if(recved<=0){
            errno = EWOULDBLOCK;
        }        
        return recved;
    }
    //阻塞读取
    timeout = 3000;
    while(recved < len){
        if(HAL_OK != HAL_UART_Receive(&uart->handle, (uint8_t*)buf + recved, 1, timeout)){
            return recved;
        }
        recved ++;
        timeout = 32;
    }        
    return recved;
}

int uart_write(int fd, const void* buf, int len){
    int writed;
    UART_Object* uart = uart_get_object(fd);
    if(!uart)return -1;
    //非阻塞写入
    if(uart->flags & O_NONBLOCK){
        writed = fifo_write(&uart->txfifo, buf, len);
        if(writed<=0){
            errno = EWOULDBLOCK;
        }
        //触发发送中断
        __HAL_UART_ENABLE_IT((&uart->handle), UART_IT_TC);
        return writed;
    }    
    //阻塞写入
    if(HAL_OK == HAL_UART_Transmit(&uart->handle, (uint8_t*)buf, len, 3000)){
        return len;
    }
    return -1;
}

//检测是否可读写
int uart_poll(int fd, int event){
    int revent = 0;
    UART_Object* uart = uart_get_object(fd);    
    if(uart->flags & O_NONBLOCK){
        if((event & POLLIN) && (!fifo_is_empty(&uart->rxfifo))){            
            revent |= POLLIN;            
        }
        if((event & POLLOUT) && fifo_is_empty(&uart->txfifo)){            
            revent |= POLLOUT;           
        }
    }else{
        revent = event;        
    }    
    return revent;
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
    int flag = uart_get_attr(fd);
    attr->c_cflag = flag;
    return (flag == 0);
}
int _tp_tcsetattr(int fd, int opt, const struct termios* attr){
    return uart_set_attr(fd, attr->c_cflag);
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

//===================== fsmc ===================


void* fsmc_init(int* pins, int count, uint32_t bank){
#ifdef GPIO_AF12_FSMC
    static SRAM_HandleTypeDef fsmc;
    FSMC_NORSRAM_TimingTypeDef Timing;
    int i;
    __HAL_RCC_FSMC_CLK_ENABLE();

    gpio_init(pins[0], GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    for(i=1;i<count;i++){
        gpio_init_ex(pins[i], GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_FSMC);
    }

    fsmc.Instance = FSMC_NORSRAM_DEVICE;
    fsmc.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

    fsmc.Init.NSBank = bank;
    fsmc.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    fsmc.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    fsmc.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    fsmc.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    fsmc.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    fsmc.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    fsmc.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    fsmc.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    fsmc.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    fsmc.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    fsmc.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    fsmc.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    //fsmc.Init.PageSize = 0;
     /* Timing */
    Timing.AddressSetupTime = 5;
    Timing.AddressHoldTime = 15;
    Timing.DataSetupTime = 8;
    Timing.BusTurnAroundDuration = 1;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;    
  
    if (HAL_SRAM_Init(&fsmc, &Timing, NULL) != HAL_OK)
    {
        return NULL;
    }
    HAL_Delay(10);
    if(bank == FSMC_NORSRAM_BANK1){
        return (void*)(uintptr_t)0x60000000;
    }
    if(bank == FSMC_NORSRAM_BANK2){
        return (void*)(uintptr_t)0x64000000;
    }
    if(bank == FSMC_NORSRAM_BANK3){
        return (void*)(uintptr_t)0x68000000;
    }
    if(bank == FSMC_NORSRAM_BANK4){
        return (void*)(uintptr_t)0x6C000000;
    } 
#endif
    return NULL;
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


typedef struct file_ops{
    read_func rd;
    write_func wr;    
    fcntl_func fctl;
    poll_func pl;
}file_ops;

static file_ops file_ops_[] = {
    {stdio_read, stdio_write, NULL, NULL},
    {gpio_read,  gpio_write,  gpio_fcntl, gpio_poll},
    {uart_read,  uart_write,  uart_fcntl, uart_poll}, 
    {spi_read,   spi_write,   NULL, NULL}, 
    {i2c_read,   i2c_write,   NULL, NULL}, 
    {rom_read,   rom_write,  NULL, NULL}, 
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


int _tp_poll(struct _tp_pollfd* fds, unsigned int nfds, int timeout){
    unsigned int i;
    int revent,type;
    int ret = 0;
    struct _tp_pollfd* pfd;
    poll_func func;
    uint32_t start = HAL_GetTick();
    do{
        for(i = 0; i<nfds; i++){
            pfd = &fds[i];
            type = FD_GET_TYPE(pfd->fd);
            func = file_ops_[type].pl;
            if(func){
                revent = func(pfd->fd, pfd->events);
            }else{
                revent = pfd->events;
            }  
            pfd->revents = (short)revent;
            if(revent)ret ++;
        }
        if(ret)break;
        //TODO 进入睡眠模式
    }while((HAL_GetTick() - start) < timeout);
    
    return ret;
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
    int type = FD_GET_TYPE(fd);
    va_list ap;
    void* value;
    int ret = -1;
    fcntl_func func = file_ops_[type].fctl;
    if(func){
        va_start(ap, cmd);
        value = va_arg(ap, void*);
        ret = func(fd, cmd, value);
        va_end(ap);
    }    
    return ret;
}




unsigned int _tp_sleep(unsigned int seconds){
    HAL_Delay(seconds * 1000);
    return 0;
}
unsigned int _tp_usleep(unsigned int micro_seconds){
   //HAL_Delay(micro_seconds / 1000);
    int ms = micro_seconds / 1000;
    if(ms){
        HAL_Delay(ms);
        return 0;
    }else{
        while(micro_seconds>0){
            micro_seconds -- ;
        }
    }
    return micro_seconds;
}

_tp_clock_t _tp_clock(){
    return HAL_GetTick();
}


// 




//============= 各系统中断 ===============

void SysTick_Handler(){ 
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler(); 
}
void EXTI0_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI1_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
void EXTI2_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
void EXTI3_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
void EXTI4_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
void EXTI9_5_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}
void EXTI15_10_IRQHandler(){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}


void USART1_IRQHandler(){
    HAL_UART_IRQHandler(&uart_objs_[0].handle);
}
#ifdef USART2
void USART2_IRQHandler(){
    HAL_UART_IRQHandler(&uart_objs_[1].handle);
}
#endif
#ifdef USART3
void USART3_IRQHandler(){
    HAL_UART_IRQHandler(&uart_objs_[2].handle);
}
#endif
#ifdef UART4
void UART4_IRQHandler(){
    HAL_UART_IRQHandler(&uart_objs_[3].handle);
}
#endif
#ifdef UART5
void UART5_IRQHandler(){
    HAL_UART_IRQHandler(&uart_objs_[4].handle);
}
#endif



#endif //#ifdef STM32
