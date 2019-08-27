#pragma once

#if defined(STM32F1)
#include "stm32f1xx.h" 
#elif defined(STM32F2) 
#include "stm32f2xx.h"
#elif defined(STM32F3) 
#include "stm32f3xx.h"
#elif defined(STM32F4) 
#include "stm32f4xx.h"
#else

#endif

#ifdef __cplusplus
extern "C" {
#endif

//fd types
#define FD_TYPE_GPIO 0x01
#define FD_TYPE_UART 0x02
#define FD_TYPE_SPI  0x03
#define FD_TYPE_I2C  0x04

//gpio port names
#define PORTA 0x00000000
#define PORTB 0x01000000
#define PORTC 0x02000000
#define PORTD 0x03000000
#define PORTE 0x04000000
#define PORTF 0x05000000
#define PORTG 0x06000000
#define PORTH 0x07000000
#define PORTI 0x08000000
#define PORTJ 0x09000000
#define PORTK 0x0A000000

//gpio pin names
#define PIN0  0x0001
#define PIN1  0x0002
#define PIN2  0x0004
#define PIN3  0x0008
#define PIN4  0x0010
#define PIN5  0x0020 
#define PIN6  0x0040
#define PIN7  0x0080
#define PIN8  0x0100 
#define PIN9  0x0200
#define PIN10 0x0400 
#define PIN11 0x0800 
#define PIN12 0x1000 
#define PIN13 0x2000 
#define PIN14 0x4000
#define PIN15 0x8000

//gpio modes
//#define  GPIO_MODE_INPUT                        0x00000000U   /*!< Input Floating Mode                   */
//#define  GPIO_MODE_OUTPUT_PP                    0x00000001U   /*!< Output Push Pull Mode                 */
//#define  GPIO_MODE_OUTPUT_OD                    0x00000011U   /*!< Output Open Drain Mode                */
//#define  GPIO_MODE_AF_PP                        0x00000002U   /*!< Alternate Function Push Pull Mode     */
//#define  GPIO_MODE_AF_OD                        0x00000012U   /*!< Alternate Function Open Drain Mode    */
//#define  GPIO_MODE_AF_INPUT                     GPIO_MODE_INPUT          /*!< Alternate Function Input Mode         */
//#define  GPIO_MODE_ANALOG                       0x00000003U   /*!< Analog Mode  */
//#define  GPIO_MODE_IT_RISING                    0x10110000U   /*!< External Interrupt Mode with Rising edge trigger detection          */
//#define  GPIO_MODE_IT_FALLING                   0x10210000U   /*!< External Interrupt Mode with Falling edge trigger detection         */
//#define  GPIO_MODE_IT_RISING_FALLING            0x10310000U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
//#define  GPIO_MODE_EVT_RISING                   0x10120000U   /*!< External Event Mode with Rising edge trigger detection               */
//#define  GPIO_MODE_EVT_FALLING                  0x10220000U   /*!< External Event Mode with Falling edge trigger detection              */
//#define  GPIO_MODE_EVT_RISING_FALLING           0x10320000U   /*!< External Event Mode with Rising/Falling edge trigger detection       */


//gpio pull
/*
GPIO_NOPULL
GPIO_PULLUP
GPIO_PULLDOWN
*/

typedef void (*irq_handler)();

void tiny_posix_init();

extern GPIO_TypeDef* gpio_ports_[];

//按port和pin获取fd，示例 GPIO_FD(PORTC, 13)
#define GPIO_FD(port, pin) (port|((1<<pin)<<8)|FD_TYPE_GPIO)

//获取反向输出的fd 
#define GPIO_FD_REVERSE(port, pin) (0x10000000|GPIO_FD(port, pin))

//fd获取port 
#define GPIO_FD_GET_PORT(fd) (gpio_ports_[(fd>>24)&0x0f])

//fd获取pin
#define GPIO_FD_GET_PIN(fd) ((fd>>8)&0xffff)

//是否反向输出  
#define GPIO_FD_IS_REVERSE(fd) (fd&0x70000000)

//gpio初始化  mode=GPIO_MODE_INPUT\GPIO_MODE_OUTPUT_PP\...  pull=GPIO_NOPULL\GPIO_PULLUP\GPIO_PULLDOWN
int gpio_init(int fd, int mode, int pull);
int gpio_init_ex(int fd, int mode, int pull, int af);

//设置中断函数
void gpio_set_irq(int fd, irq_handler func);

//read/write 函数
int gpio_read(int fd, void* buf, int len);
int gpio_write(int fd, const void* buf, int len);

//gpio 操作
#define gpio_status(fd) HAL_GPIO_ReadPin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd))
#define gpio_set(fd) HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), (0 == GPIO_FD_IS_REVERSE(fd)))
#define gpio_reset(fd) HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), (0 != GPIO_FD_IS_REVERSE(fd)))
#define gpio_toggle(fd) HAL_GPIO_TogglePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd))

//============================= uart =============================

//uart fd   id 1~5
#define UART_FD(id) ((id-1)<<8|FD_TYPE_UART)

//uart 默认引脚 (TX RX)
#define UART1_DEFAULT_PINS GPIO_FD(PORTA, 9),  GPIO_FD(PORTA, 10)
#define UART2_DEFAULT_PINS GPIO_FD(PORTA, 2),  GPIO_FD(PORTA, 3)
#define UART3_DEFAULT_PINS GPIO_FD(PORTB, 10), GPIO_FD(PORTB, 11)
#define UART4_DEFAULT_PINS GPIO_FD(PORTC, 10), GPIO_FD(PORTC, 11)
#define UART5_DEFAULT_PINS GPIO_FD(PORTC, 12), GPIO_FD(PORTD, 2)

//初始化串口， flags示例 B9600|CS8
int uart_init(int fd, int txpin, int rxpin, int flags);

//修改串口参数， flags示例 B9600|CS8
int uart_set_flags(int fd, int flags);

//read/write 函数
int uart_read(int fd, void* buf, int len);
int uart_write(int fd, const void* buf, int len);


//============================= spi =============================

//spi fd   id 1~3
#define SPI_FD(id) ((id-1)<<8|FD_TYPE_SPI)

//spi 默认引脚 (CLK MISO MOSI)
#define SPI1_DEFAULT_PINS  GPIO_FD(PORTA, 5),  GPIO_FD(PORTA, 6),  GPIO_FD(PORTA, 7)
#define SPI2_DEFAULT_PINS  GPIO_FD(PORTB, 13), GPIO_FD(PORTB, 14), GPIO_FD(PORTB, 15)
#define SPI3_DEFAULT_PINS  GPIO_FD(PORTB, 3),  GPIO_FD(PORTB, 4),  GPIO_FD(PORTB, 5)


//初始化spi(主机)，flags=0
int spi_init(int fd, int clkpin, int misopin, int mosipin);

//写len个字节，并读len个字节， 成功返回len， 失败返回<=0
int spi_io(int fd, const void* out, void* in, int len);

//read/write 函数
int spi_read(int fd, void* buf, int len);
int spi_write(int fd, const void* buf, int len);


//============================= i2c =============================

//i2c fd   id 1~2
#define I2C_FD(id) ((id-1)<<8|FD_TYPE_I2C)


//i2c 默认引脚 (SCL SDA)
#define I2C1_DEFAULT_PINS  GPIO_FD(PORTB, 6),  GPIO_FD(PORTB, 7)
#define I2C2_DEFAULT_PINS  GPIO_FD(PORTB, 13), GPIO_FD(PORTB, 14)


//初始化i2c（主机）
int i2c_init(int fd, int sclpin, int sdapin, int addr_and_flags);

//设置之后通信的对端地址（从机地址）
int i2c_set_peer(int fd, int addr);
//设置本机地址
int i2c_set_local(int fd, int addr);

//read/write 函数
int i2c_read(int fd, void* buf, int len);
int i2c_write(int fd, const void* buf, int len);

#ifdef __cplusplus
}
#endif


