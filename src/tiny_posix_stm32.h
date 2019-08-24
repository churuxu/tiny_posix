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

//port names
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

//pin names
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

typedef void (*irq_handler)();

extern GPIO_TypeDef* gpio_ports_[];

//按port和pin获取fd，示例 GPIO_FD(PORTC, PIN13)
#define GPIO_FD(port, pin) (port|(pin<<8)|FD_TYPE_GPIO)

//获取反向输出的fd 
#define GPIO_FD_REVERSE(port, pin) (0x10000000|port|(pin<<8)|FD_TYPE_GPIO)

//fd获取port 
#define GPIO_FD_GET_PORT(fd) (gpio_ports_[(fd>>24)&0x0f])

//fd获取pin
#define GPIO_FD_GET_PIN(fd) ((fd>>8)&0xffff)

//是否反向输出  
#define GPIO_FD_IS_REVERSE(fd) (fd&0x70000000)


#define gpio_status(fd)  HAL_GPIO_ReadPin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd))

#define gpio_set(fd) HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), (0 == GPIO_FD_IS_REVERSE(fd)))

#define gpio_reset(fd) HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), (0 != GPIO_FD_IS_REVERSE(fd)))

#define gpio_toggle(fd) HAL_GPIO_TogglePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd))

void gpio_set_irq(int fd, irq_handler func);

/*
static int  gpio_status(int fd){
    return HAL_GPIO_ReadPin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd));
}
static void gpio_set(int fd){
    HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), 1);
}
static void gpio_reset(int fd){
    HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), 0);
}
static void gpio_toggle(int fd){
    HAL_GPIO_TogglePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd));
}
*/

#define UART_FD(id) ((id-1)<<8|FD_TYPE_UART)

#ifdef __cplusplus
}
#endif


