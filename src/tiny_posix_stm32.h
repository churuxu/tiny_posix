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

#include "lcd.h"

#define _tp_open _open 
#define _tp_close _close
#define _tp_read _read
#define _tp_write _write

#ifdef __cplusplus
extern "C" {
#endif

//fd types
#define FD_TYPE_STD   0x00000000 //stdio
#define FD_TYPE_GPIO  0x01000000
#define FD_TYPE_UART  0x02000000
#define FD_TYPE_SPI   0x03000000
#define FD_TYPE_I2C   0x04000000
#define FD_TYPE_LCD   0x05000000
//#define FD_TYPE_FLASH 0x05000000 //内部flash


#define FD_GET_TYPE(fd) (fd>>24)

typedef void (*irq_handler)();

void tiny_posix_init();

//修改stdout stdin stderr
void stdio_set_fd(int in, int out, int err);


//gpio port names
#define PORTA 0x000000
#define PORTB 0x010000
#define PORTC 0x020000
#define PORTD 0x030000
#define PORTE 0x040000
#define PORTF 0x050000
#define PORTG 0x060000
#define PORTH 0x070000
#define PORTI 0x080000
#define PORTJ 0x090000
#define PORTK 0x0A0000


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


extern GPIO_TypeDef* gpio_ports_[];

//按port和pin获取fd，示例 GPIO_FD(PORTC, 13)
#define GPIO_FD(port, pin) (port|(1<<pin)|FD_TYPE_GPIO)

#define GPIO_FD_REVERSE_FLAG 0x100000

//获取反向输出的fd 
#define GPIO_FD_REVERSE(port, pin) (GPIO_FD_REVERSE_FLAG|GPIO_FD(port, pin))

//fd获取port 
#define GPIO_FD_GET_PORT(fd) (gpio_ports_[(fd>>16)&0x0f])

//fd获取pin
#define GPIO_FD_GET_PIN(fd) (fd&0xffff)

//是否反向输出  
#define GPIO_FD_IS_REVERSE(fd) (fd&GPIO_FD_REVERSE_FLAG)

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
#define UART_FD(id) ((id-1)|FD_TYPE_UART)

#define UART_FD_GET_INDEX(fd) (fd&0x0f)

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
#define SPI_FD(id) ((id-1)|FD_TYPE_SPI)

#define SPI_FD_GET_INDEX(fd) (fd&0x0f)

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
#define I2C_FD(id) ((id-1)|FD_TYPE_I2C)

#define I2C_FD_GET_INDEX(fd) (fd&0x0f)

//i2c 默认引脚 (SCL SDA)
#define I2C1_DEFAULT_PINS  GPIO_FD(PORTB, 6),  GPIO_FD(PORTB, 7)
#define I2C2_DEFAULT_PINS  GPIO_FD(PORTB, 13), GPIO_FD(PORTB, 14)


//初始化i2c（主机）
int i2c_init(int fd, int sclpin, int sdapin, int addr_and_flags, int peeraddr);

//设置之后通信的对端地址（从机地址）
int i2c_set_peer(int fd, int addr);
//设置本机地址
int i2c_set_local(int fd, int addr);

//设置读写地址
int i2c_set_offset(int fd, int offset);

int i2c_test(int fd);

//read/write 函数
int i2c_read(int fd, void* buf, int len);
int i2c_write(int fd, const void* buf, int len);



//===================== lcd ===================
//id=1~2
#define LCD_FD(id) ((id-1)|FD_TYPE_LCD)
#define LCD_FD_GET_INDEX(fd) (fd&0x0f)


int lcd_init(int fd, LCD_DrvTypeDef* driver, uint16_t lcdid);

//turn on/off
int lcd_display(int fd, int on);

int lcd_get_width(int fd);
int lcd_get_height(int fd);

int lcd_draw(int fd, int x, int y, int w, int h, void* rgbdata);

#ifdef __cplusplus
}
#endif


