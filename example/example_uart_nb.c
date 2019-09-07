#include "tiny_posix.h"

//非阻塞示例
#ifdef _WIN32
#define PORT_NAME "COM8"
#define ECHO 0
#else
#define PORT_NAME "/dev/ttyS0"
#define ECHO 1
#endif



void example_uart_nb(){
    int fd, ret;
    char buf[65];
    struct pollfd fds[1];
    struct termios attr;

    fd = open(PORT_NAME, O_RDWR|O_NONBLOCK);
    if(fd<0){
        printf("uart open error\n");
        return;
    }else{
        printf("uart opened\n");
    }

    ret = tcgetattr(fd, &attr);
    if(ret){
        printf("tcgetattr error\n");
        return;        
    }

    cfsetispeed(&attr, B9600);
    cfsetospeed(&attr, B9600);

    attr.c_cflag &= ~CSIZE;
    attr.c_cflag |= CS8;
    attr.c_cflag &= ~PARENB;
    attr.c_cflag &= ~PARODD;
    attr.c_cflag &= ~CSTOPB;
    ret = tcsetattr(fd, TCSANOW,  &attr);
    if(ret){
        printf("tcsetattr error\n");
        return;        
    }

    fds[0].fd = fd;
    fds[0].events = POLLIN;
    while(1){ 
        fds[0].revents = 0;
        ret = poll(fds, 1, 3000);
        if(ret>0 && fds[0].revents & POLLIN){
            ret = read(fd, buf, 64);
            if(ret>0){
                printf("uart recved %d bytes\n", ret);
                if(ECHO){
                    write(fd, buf, ret);
                }else{
                    buf[ret] = 0;
                    printf("%s\n", buf);
                }
            }else{
                printf("uart recved error\n");
            }
        }else{
            printf("no data recved\n");
            write(fd, "hello", 5);
        }
    }
    
}



