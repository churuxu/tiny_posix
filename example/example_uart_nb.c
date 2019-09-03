#include "tiny_posix.h"

//非阻塞示例


void example_uart_nb(){
    int fd, ret;
    char buf[64];
    struct pollfd fds[1];

    fd = open("/dev/ttyS0", O_RDWR|O_NONBLOCK);
    if(fd<0){
        printf("uart open error\n");
        return;
    }else{
        printf("uart opened\n");
    }
    fds[0].fd = fd;
    fds[0].events = POLLIN;
    while(1){ 
        ret = poll(fds, 1, 3000);
        if(ret>0 && fds[0].revents & POLLIN){
            ret = read(fd, buf, 64);
            if(ret>0){
                printf("uart recved %d bytes\n", ret);
                write(fd, buf, ret);
            }else{
                printf("uart recved error\n");
            }
        }else{
            printf("no data recved\n");
            write(fd, "hello", 5);
        }
    }
    
}



