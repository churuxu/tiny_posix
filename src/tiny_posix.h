#pragma once

/*

user layer:
http://pubs.opengroup.org/onlinepubs/9699919799/


功能：
文件
Socket
串口
目录
时钟
事件
动态库

*/


//win32 headers
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#endif

//headers
#ifdef __GNUC__
#include <fcntl.h> 
#include <unistd.h> 
#endif

//stdc headers
#include <stdlib.h> //for size_t
#include <stdint.h>
#include <stdio.h> //for SEEK_SET
#include <string.h> //for malloc free
#include <errno.h>
#include <signal.h> 
#include <time.h> 

//generic posix headers
#if defined(__linux__)
#include <unistd.h> //open close read write
#include <poll.h>  //poll
#include <fcntl.h> //fcntl
#include <termios.h> 
#include <sys/socket.h> 
#endif

#ifdef STM32
#include "tiny_posix_stm32.h"
#endif

#ifdef _WIN32
#include "tiny_posix_win32.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__linux__)

//============== types ===============

typedef int posix_ssize_t;
typedef uint16_t posix_speed_t;
typedef uint64_t posix_off_t;
#if __SIZE_OF_POINTER__ == 64 
typedef uint64_t posix_clock_t;
#else
typedef uint32_t posix_clock_t;
#endif
typedef uint64_t posix_time_t;
typedef int posix_socklen_t;
typedef struct posix_DIR posix_DIR;
struct posix_timeval{
    posix_time_t tv_sec;
    long tv_usec;
};

struct posix_timezone{
    int tz_minuteswest;
    int tz_dsttime;
};

struct posix_pollfd {
    int fd;
    short events;     
    short revents;
};



struct posix_sockaddr{
    unsigned short sa_family;
    char sa_data[14];
};

struct posix_stat{
    unsigned short sa_family;
    char sa_data[14];
};

struct posix_termios{
    uint32_t c_cflag;
};

struct posix_utimbuf{
    unsigned short sa_family;
    char sa_data[14];
};
//============== defines ===============

//for access()
#ifndef F_OK
#define F_OK  1 
#define R_OK  2 
#define W_OK  4 
#define X_OK  8 
#endif

//for poll()
#ifndef POLLIN
#define POLLIN  1 //有数据可读时触发
#define POLLOUT 2 //网络连接上时触发
#define POLLHUP 4 //网络连接断开时触发
#define POLLERR 8  //io出错
#define POLLPRI 16 //高优先数据到来
#endif

#ifndef F_GETFL
#define F_GETFL 0
#define F_SETFL 1
#endif

#ifndef O_NONBLOCK
#define O_NONBLOCK 0xf0
#endif

#undef EWOULDBLOCK
#define EWOULDBLOCK EAGAIN

//============== 串口 ================

//波特率
#define B300    3
#define B600    6
#define B1200   12
#define B2400   24
#define B4800   48
#define B9600   96
#define B19200  192
#define B38400  384
#define B57600  576
#define B115200 1152

//数据位
#define CS5   0x050000
#define CS6   0x060000
#define CS7   0x070000
#define CS8   0x080000
#define CSIZE 0xff0000

//停止位
#define CSTOPB 0x01000000 //有=2 无=1

//校验位   PARENB|PARODD=奇校验  PARENB=偶校验
#define PARENB 0x02000000 
#define PARODD 0x04000000

//修改串口参数
#define TCSANOW 0
#define TCSADRAIN 1
#define TCSAFLUSH 2



//============== renames ===============
#ifndef TINY_POSIX_IMPL
#define off_t posix_off_t
#define time_t posix_time_t
#define clock_t posix_clock_t
#define pollfd posix_pollfd
#define timeval posix_timeval
#define timezone posix_timezone
#define socklen_t posix_socklen_t
#define DIR posix_DIR
#define speed_t posix_speed_t
#define sockaddr posix_sockaddr
#define stat posix_stat
#define termios posix_termios
#define utimbuf posix_utimbuf

#define open posix_open
#define close posix_close
#define poll posix_poll
#define read posix_read
#define write posix_write
#define fcntl posix_fcntl
#define lseek posix_lseek

#define socket posix_socket
#define connect posix_connect
#define listen posix_listen
#define accept posix_accept
#define bind posix_bind
#define sendto posix_sendto
#define recvfrom posix_recvfrom
#define recv posix_recv
#define send posix_send

#define clock posix_clock
#define gettimeofday posix_gettimeofday
#define settimeofday posix_settimeofday

#define sleep posix_sleep
#define usleep posix_usleep
#endif

//============== generic ===============
ssize_t read(int fd, void * buf, size_t count);
ssize_t write(int fd, const void* buf, size_t count);
int poll(struct pollfd* fds, unsigned int nfds, int timeout);
int close(int fd);
int open(const char* pathname, int flags, ...);
int fcntl(int fd, int cmd, ...);
off_t lseek(int fd, off_t offset, int where);


//========== socket ===========
int socket(int af, int type, int proto);
int connect(int fd, const struct sockaddr* addr, socklen_t addrlen);
int accept(int fd, struct sockaddr* addrbuf, socklen_t* addrlen);
int bind(int fd, const struct sockaddr* addr, socklen_t addrlen);
int listen(int fd, int cap);
ssize_t recv(int fd, void* buf, size_t count, int flags);
ssize_t send(int fd, const void* buf, size_t count, int flags);
ssize_t recvfrom(int fd, void* buf, size_t buflen, int flags, struct sockaddr* addrbuf, socklen_t* addrlen);
ssize_t sendto(int fd, const void* buf, size_t buflen, int flags, const struct sockaddr* addr, socklen_t addrlen);


//========== date time ===========
clock_t clock(void);
int gettimeofday(struct timeval* tv, struct timezone* tz);
int settimeofday(const struct timeval* tv, const struct timezone* tz);


//========== sys ===========
unsigned int sleep(unsigned int seconds);
unsigned int usleep(unsigned int micro_seconds);


//========== dlfcn ===========
void* dlopen(const char* name, int flags);
int dlclose(void* handle);
void* dlsym(void* handle, const char* funcname);


//======= serial port ============
int cfsetispeed(struct termios* attr, speed_t t);
int cfsetospeed(struct termios* attr, speed_t t);
int tcgetattr(int fd, struct termios* attr);
int tcsetattr(int fd, int opt, const struct termios* attr);




#endif //defined(__linux__)

#ifdef __cplusplus
}
#endif

