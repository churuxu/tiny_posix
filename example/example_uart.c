#include "tiny_posix.h"




void example_uart(){
    int fd, ret;
    char buf[64];

    fd = open("/dev/ttyS0", O_RDWR);
    if(fd<0){
        printf("uart open error\n");
        return;
    }else{
        printf("uart opened\n");
    }

    while(1){        
        //sleep(1);
        ret = read(fd, buf, 64);
        if(ret>0){
            printf("uart recved %d bytes\n", ret);
            write(fd, buf, ret);
        }else{
            write(fd, "hello", 5);
        }
    }
    
}



