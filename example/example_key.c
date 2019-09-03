#include "tiny_posix.h"

static int ledfd_;
static int btnfd_;

void on_key(){
	uint8_t v = 0;
    read(btnfd_, &v, 1);
    if(v == 0){ //释放
        write(ledfd_, &v, 1);
    }else{ //按下
        write(ledfd_, &v, 1);
    }
}


void example_key(){    
	ledfd_ = open("/leds/led0", O_WRONLY);    
    if(ledfd_ <0){
        printf("open led error\n");
    }

    btnfd_ = open("/keys/key0", O_RDONLY);
    if(btnfd_ <0){
        printf("open key error\n");
    }

    gpio_set_irq(btnfd_, on_key);
    //signal(SIGIO, on_key);
    printf("press key to toggle led\n");

    while(1){
        sleep(1);        
    }
    

}



