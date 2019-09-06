#pragma once


#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>

#ifdef __GNUC__
//mingw
#include <unistd.h> 
#endif

#ifdef _MSC_VER
//vc
#if __SIZE_OF_POINTER__ == 64 
typedef int64_t ssize_t;
#else
typedef int32_t ssize_t;
#endif
typedef int socklen_t;

#endif



#define USE_SYSTEM_GETADDRINFO 
#define USE_SYSTEM_CLOCK

#ifdef _MSC_VER
#define USE_SYSTEM_POLLFD
#endif


#undef lseek
