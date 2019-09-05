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


#define posix_open _open 
#define posix_close _close
#define posix_read _read
#define posix_write _write

#ifdef __cplusplus
extern "C" {
#endif

//fd types
#define FD_TYPE_STD   0x00000000 //stdio
#define FD_TYPE_GPIO  0x01000000
#define FD_TYPE_UART  0x02000000
#define FD_TYPE_SPI   0x03000000
#define FD_TYPE_I2C   0x04000000
#define FD_TYPE_ROM   0x05000000 //内部flash，只读
#define FD_TYPE_DISK  0x06000000 //外部flash、sdcard等，由驱动实现
#define FD_TYPE_LCD   0x07000000
//#define FD_TYPE_FLASH 0x05000000 //内部flash


#define FD_GET_TYPE(fd) (fd>>24)



typedef void (*irq_handler)();
typedef int (*read_func)(int fd, void* buf, int len);
typedef int (*write_func)(int fd, const void* buf, int len);
typedef int (*fcntl_func)(int fd, int cmd, void* v);
typedef int (*poll_func)(int fd, int event);

void tiny_posix_init();

//修改stdout stdin stderr
void stdio_set_fd(int in, int out, int err);
void stdio_set_func(read_func in, write_func out, write_func err);

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

//示例 GPIO_MULTI_FD(PORTC, GPIO_PIN_1|GPIO_PIN_2)
#define GPIO_MULTI_FD(port, pins) (port|pins|FD_TYPE_GPIO)

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
int gpio_fcntl(int fd, int key, void* value);
int gpio_poll(int fd, int event);

//gpio 操作
#define gpio_status(fd)  ((0 == GPIO_FD_IS_REVERSE(fd)) == HAL_GPIO_ReadPin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd)))
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

//初始化串口， attr示例 B9600|CS8
int uart_init(int fd, int txpin, int rxpin, int attr);

//修改串口参数， attr示例 B9600|CS8
int uart_set_attr(int fd, int attr);

//read/write 函数
int uart_read(int fd, void* buf, int len);
int uart_write(int fd, const void* buf, int len);
int uart_fcntl(int fd, int cmd, void* val);
int uart_poll(int fd, int event);

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

//===================== rom ===================
//ROM_FD(0x07ff) = 0x080007ff
#define ROM_FD(addr) ((addr)|FD_TYPE_ROM)
#define ROM_FD_GET_ADDRESS(fd) ((void*)(uintptr_t)((fd & 0xffffff)|0x08000000))


//read/write 函数
int rom_read(int fd, void* buf, int len);
int rom_write(int fd, const void* buf, int len);


//===================== fsmc ===================

void* fsmc_init(int* pins, int count, uint32_t bank);


//===================== lcd ===================
//id=1~2
#define LCD_FD(index) ((index)|FD_TYPE_LCD)
#define LCD_FD_GET_INDEX(fd) (fd&0x0f)


typedef struct
{
  void     (*Init)(void);
  uint16_t (*ReadID)(void);
  void     (*DisplayOn)(void);
  void     (*DisplayOff)(void);
  void     (*SetCursor)(uint16_t, uint16_t);
  void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
  uint16_t (*ReadPixel)(uint16_t, uint16_t);
  
   /* Optimized operation */
  void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  
  uint16_t (*GetLcdPixelWidth)(void);
  uint16_t (*GetLcdPixelHeight)(void);
  void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
  void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*);
}LCD_DrvTypeDef; 

int lcd_init(int fd, LCD_DrvTypeDef* driver, uint16_t lcdid);

//turn on/off
int lcd_display(int fd, int on);

int lcd_get_width(int fd);
int lcd_get_height(int fd);

int lcd_fill(int fd, int x, int y, int w, int h, uint32_t color);
int lcd_draw(int fd, int x, int y, int w, int h, void* rgbdata);


//===================== raw diskio ===================
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */

#define CTRL_SYNC 0
#define GET_SECTOR_COUNT	1	/* Get media size (needed at _USE_MKFS == 1) */
#define GET_SECTOR_SIZE		2	/* Get sector size (needed at _MAX_SS != _MIN_SS) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (needed at _USE_MKFS == 1) */

typedef struct{
  int (*disk_initialize) (uint8_t id);                     /*!< Initialize Disk Drive                     */
  int (*disk_status)     (uint8_t id);                     /*!< Get Disk Status                           */
  DRESULT (*disk_read)       (uint8_t id, uint8_t* buf, unsigned int sector, unsigned int count);       /*!< Read Sector(s)                            */
  DRESULT (*disk_write)      (uint8_t id, const uint8_t* buf, unsigned int sector, unsigned int count); /*!< Write Sector(s) when _USE_WRITE = 0       */
  DRESULT (*disk_ioctl)      (uint8_t id, uint8_t cmd, void* buf);              /*!< I/O control operation when _USE_IOCTL = 1 */
}Diskio_drvTypeDef;

//id 1~16  
#define DISK_FD(index) ((index)|FD_TYPE_DISK)
#define DISK_FD_GET_INDEX(fd) (fd&0x0f)

//初始化一个外部存储为disk
int disk_init(int fd, Diskio_drvTypeDef* driver);


int disk_lseek(int fd, int offset, int how);
int disk_read(int fd, void* buf, int len);
int disk_write(int fd, const void* buf, int len);


#ifdef __cplusplus
}
#endif


