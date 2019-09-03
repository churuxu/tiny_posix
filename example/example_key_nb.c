#include "tiny_posix.h"

/*
非阻塞按键事件
*/

static int ledfd_;
static int btnfd_;

void example_key_nb(){  
    struct pollfd fds[1];
    uint8_t v;
    int ret;

	ledfd_ = open("/leds/led0", O_WRONLY);    
    if(ledfd_ <0){
        printf("open led error\n");
    }

    btnfd_ = open("/keys/key0", O_RDONLY);
    if(btnfd_ <0){
        printf("open key error\n");
    }
    
    printf("press key to toggle led\n");

    fds[0].fd = btnfd_;
    fds[0].events = POLLPRI;
    while(1){
        ret = poll(fds, 1, 5000);
        if(ret>0){
            if(fds[0].revents & POLLPRI){
                v = 0;
                read(btnfd_, &v, 1);
                write(ledfd_, &v, 1);
            }
        }
    }
    

}



