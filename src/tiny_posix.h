#pragma once

/*

user layer:
http://pubs.opengroup.org/onlinepubs/9699919799/

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


#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__linux__)

//============== types ===============

typedef int _tp_ssize_t;
typedef uint16_t _tp_speed_t;
typedef uint64_t _tp_off_t;
#if __SIZE_OF_POINTER__ == 64 
typedef uint64_t _tp_clock_t;
#else
typedef uint32_t _tp_clock_t;
#endif
typedef uint64_t _tp_time_t;
typedef int _tp_socklen_t;
typedef struct _tp_DIR _tp_DIR;
struct _tp_timeval{
    _tp_time_t tv_sec;
    long tv_usec;
};

struct _tp_timezone{
    int tz_minuteswest;
    int tz_dsttime;
};

struct _tp_pollfd {
    int fd;
    short events;     
    short revents;
};



struct _tp_sockaddr{
    unsigned short sa_family;
    char sa_data[14];
};

struct _tp_stat{
    unsigned short sa_family;
    char sa_data[14];
};

struct _tp_termios{
    uint32_t c_cflag;
};

struct _tp_utimbuf{
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
#define off_t _tp_off_t
#define time_t _tp_time_t
#define clock_t _tp_clock_t
#define pollfd _tp_pollfd
#define timeval _tp_timeval
#define timezone _tp_timezone
#define socklen_t _tp_socklen_t
#define DIR _tp_DIR
#define speed_t _tp_speed_t
#define sockaddr _tp_sockaddr
#define stat _tp_stat
#define termios _tp_termios
#define utimbuf _tp_utimbuf

#define open _tp_open
#define close _tp_close
#define poll _tp_poll
#define read _tp_read
#define write _tp_write
#define fcntl _tp_fcntl
#define lseek _tp_lseek

#define socket _tp_socket
#define connect _tp_connect
#define listen _tp_listen
#define accept _tp_accept
#define bind _tp_bind
#define sendto _tp_sendto
#define recvfrom _tp_recvfrom
#define recv _tp_recv
#define send _tp_send

#define clock _tp_clock
#define gettimeofday _tp_gettimeofday
#define settimeofday _tp_settimeofday

#define sleep _tp_sleep
#define usleep _tp_usleep


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


//=========== file system ==============
int stat(const char* filename, struct stat* out);
int mkdir(const char* path, mode_t mode);
int rmdir(const char* path);
int unlink(const char* filename);
int utime(const char* filename, const struct utimbuf* out);
int access(const char* filename, int mode);

DIR* opendir(const char* name);
int closedir(DIR* d);
struct dirent* readdir(DIR* d);


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

