#include "tiny_posix.h"



static int leds_[8];
static int ledcount_;

void loop_leds_step(){
    static uint8_t on = 1;
    static int cur = 0; 
    if(!ledcount_) return ;
      
    write(leds_[cur], &on, 1);
    
    cur ++;
    if(cur == ledcount_){
        cur = 0;
        on = (on == 0);
    }
}


void open_leds(){
    char buf[64];
    int i, fd;   
    for(i=0;i<8;i++){
        snprintf(buf, 64, "/leds/led%d",i);
        fd = open(buf, O_WRONLY);
        if(fd>=0){
            leds_[i] = fd;
            ledcount_ ++;
        }
    }
}


void example_led(){
    open_leds();

    while(1){        
        sleep(1);
        loop_leds_step();
    }
    

}



