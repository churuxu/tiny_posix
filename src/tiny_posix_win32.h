#pragma once



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

#ifdef __cplusplus
extern "C" {
#endif

/*
#define O_NONBLOCK 0x00010000


#define POLLIN  1 //有数据可读时触发
#define POLLOUT 2 //网络连接上时触发
#define POLLHUP 4 //网络连接断开时触发
#define POLLERR 8  //io出错
#define POLLPRI 16 //高优先数据到来


struct posix_pollfd{
    int fd;
    short events;
    short revents;
};

int posix_poll(struct posix_pollfd* pfds, int nfds, int timeout);
*/

#ifndef TINY_POSIX_IMPL


#endif



#ifdef __cplusplus
}
#endif



