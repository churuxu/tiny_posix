#include "tiny_posix.h"




int example_rtc(){
	struct timeval tv;
	struct tm* t;
	time_t tim;
	char buf[64];
	//gettimeofday(&tv, NULL);
	while(1){
		time(&tim);
		t = localtime(&tim);
		int ret = strftime(buf, 64, "%Y-%m-%d %H:%M:%S", t);
		printf("%s\n",buf);
		sleep(1);
	}
	return 0;
}
