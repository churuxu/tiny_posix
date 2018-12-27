#pragma once

#include <stdlib.h> //for size_t
#include <stdint.h>
#include <stdio.h> //for SEEK_SET
#include <string.h> //for malloc free
#include <errno.h>
#include <signal.h> 
#include <time.h> 
#include <unistd.h> 

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
备注：
宏名称作为标准，宏的值可以自行修改
函数实现也可以改成宏实现
*/


struct pollfd_win32 {
    int fd;           /* 文件描述符（输入参数） */
    short events;     /* 等待的事件（输入参数） */
    short revents;    /* 实际发生了的事件（输出参数） */
}pollfd_win32;

//函数api列表
int errno_win32();
int poll_win32(struct pollfd_win32* fds, unsigned int nfds, int timeout);
ssize_t read_win32(int fd, void* buf, size_t buflen);
ssize_t write_win32(int fd, const void* buf, size_t buflen);
int close_win32(int fd);
int socket_win32(int af, int type, int proto);
int fcntl_win32(int fd, int type, int v);
int connect_win32(int fd, void* addr, int addrlen);
int reboot_win32(int how);

//宏列表
#ifndef RUNTIME_IMPL
//#define main app_startup
#undef errno
#define errno  errno_win32()
#define poll poll_win32
#define pollfd pollfd_win32
#define read read_win32
#define write write_win32
#define socket socket_win32
#define connect connect_win32
#define close close_win32
#define fcntl fcntl_win32
#define reboot reboot_win32
#endif

//IO事件定义, 用于poll()轮询事件的设置和返回
#ifndef POLLIN
#define POLLIN  1 //有数据可读时触发
#define POLLOUT 2 //网络连接上时触发
#define POLLHUP 4 //网络连接断开时触发
#define POLLERR 8  //io出错
#define POLLPRI 16 //高优先数据到来（电表发生xxx紧急事件）
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

#ifdef __cplusplus
}
#endif

