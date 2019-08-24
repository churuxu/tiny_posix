#pragma once


#ifdef __cplusplus
extern "C" {
#endif


#define GPIO_FLAGS_OUTPUT     0x0000
#define GPIO_FLAGS_INPUT      0x0100

#define GPIO_FLAGS_NO_PULL    0x00
#define GPIO_FLAGS_PULL_UP    0x01
#define GPIO_FLAGS_PULL_DOWN  0x02


//
int tiny_posix_init();
int fd_mount(const char* name, int fd);

int gpio_init(int fd, int flags);
int gpio_config(int fd, int key, void* value);
int gpio_read(int fd, void* buf, int len);
int gpio_write(int fd, const void* buf, int len);

int uart_init(int fd, int flags);
int uart_config(int fd, int key, void* value);
int uart_read(int fd, void* buf, int len);
int uart_write(int fd, const void* buf, int len);

int spi_init(int fd);
int spi_config(int fd, int key, void* value);
int spi_read(int fd, void* buf, int len);
int spi_write(int fd, const void* buf, int len);

#ifdef __cplusplus
}
#endif