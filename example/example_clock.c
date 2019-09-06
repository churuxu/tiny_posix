#include "tiny_posix.h"


uint16_t get_tick(){
	return (uint16_t)(clock() * 1000 / CLOCKS_PER_SEC);
}


static uint16_t startat_;
static uint16_t delay_;

static void add_timer(uint16_t delay){
	startat_ = get_tick();
	delay_ = delay;	
}



static void use_time_func(int ms){
	usleep(ms*1000);
}

static void process_timer(){
	uint16_t at = startat_ + delay_;
	uint16_t now = get_tick();
	uint16_t trigger = 0;
	if(at < startat_){ //溢出
		if(now < startat_ && now >= at){
			trigger = 1;
		}
	}else{ //未溢出
		if(now >= at || now < startat_){
			trigger = 1;
		}
	}

	if(trigger){
		printf("do timer at %d\n", (int)get_tick());
		add_timer(3000);
		use_time_func(13);
	}
}




int example_clock(){	
	printf("start at %d\n", (int)get_tick());
	add_timer(3000);
	while(1){
		use_time_func(13);
		process_timer();
		usleep(5000);
	}
	return 0;
}
