
/*
此文件需要能在Linux下编译通过

在不修改此文件的情况下，
使用自定义API头文件，能编译通过，则说明自定义API头文件OK
*/
#include <stdint.h>
#include <stdlib.h> //内存/字符串等
#include <string.h> //内存/字符串等
#include <assert.h> //assert
#include <errno.h> //错误码
#include <signal.h> //信号处理

#include <stdio.h> //fopen fread fwrite
#include <unistd.h> //open close read write等
#include <poll.h>  //poll
#include <fcntl.h> //fcntl
#include <time.h> //clock
#include <sys/socket.h> //socket
#include <sys/reboot.h> //reboot



int test_file_stdc(){
    int ret;
    char buf[16];
    FILE* f;    
   
    //错误码测试
    f = fopen("a.txt", "rb");
    assert(f == NULL); 
    assert(errno == ENOENT);

    //写文件测试
    f = fopen("b.txt","wb");
    
    assert(f != NULL);
    ret = fwrite("hello", 1, 5, f);
    assert(ret == 5);
    fclose(f);

    //读文件测试
    f = fopen("b.txt","rb");
    assert(f != NULL);
    ret = fread(buf, 1, 1, f);
    assert(ret == 1);
    assert(memcmp("h",buf,1) == 0);
    ret = fread(buf, 1, 16, f);
    assert(ret == 4);
    assert(memcmp("ello",buf,4) == 0);
    fseek(f, 2, SEEK_SET);
    ret = fread(buf, 1, 16, f);
    assert(ret == 3);
    assert(memcmp("llo",buf,3) == 0);

    fclose(f);
    return 0;
}


/*
int test_file_posix(){
    int ret;
    char buf[16];
    int f;
    int e;
    //错误码测试
    f = open("a.txt", O_RDONLY);
    assert(f < 0); 
    assert(errno == ENOENT);

    //写文件测试
    f = open("b.txt",O_WRONLY|O_CREAT|O_TRUNC);
    e = errno;
    assert(f > 0);
    ret = write(f, "hello", 5);
    assert(ret == 5);
    close(f);

    //读文件测试
    f = open("b.txt",O_RDONLY);
    assert(f > 0);
    ret = read(f, buf, 1);
    assert(ret == 1);
    assert(memcmp("h",buf,1) == 0);
    ret = read(f, buf, 16);
    assert(ret == 4);
    assert(memcmp("ello",buf,4) == 0);
    lseek(f, 2, SEEK_SET);
    ret = read(f, buf, 16);
    assert(ret == 3);
    assert(memcmp("llo",buf,3) == 0);

    close(f);  
    return 0;
}
*/



void on_term(int sig){
    //收到xxx信号时处理    
}

void test_signal(){
    //设置信号处理函数   SIGTERM: 程序即将退出 
    signal(SIGTERM, on_term);
}

void test_mem(){
    char* str = (char*)malloc(1024);
    assert(str != NULL);
    free(str);
}

void test_reboot(){
    //reboot(0);
}
/*
void test_clock_linux(){
    struct timespec ts;
    int ret = clock_gettime(CLOCK_MONOTONIC,&ts);
}
*/
int test_clock_stdc(){
    clock_t t = clock();
	int t1 = t / CLOCKS_PER_SEC;
	return t1;
}


void test_socket(){
    char buf[16];
    uint8_t addrbuf[] = {2,0, 0x23,0x29, 127,0,0,1, 0,0,0,0,0,0,0,0}; //127.0.0.1:9001
    int ret;
    struct pollfd fds[1];
    
    //创建连接
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    assert(sock >= 0);

    //设置非阻塞
    ret = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, ret | O_NONBLOCK);  

    //连接服务器
    ret = connect(sock, (struct sockaddr*)addrbuf, sizeof(addrbuf));
    assert(ret == 0 || (ret < 0 && errno == EWOULDBLOCK));  //返回连接成功或正在连接

    if(ret < 0 && errno == EWOULDBLOCK){ //正在连接
        //等待可写
        fds[0].fd = sock;
        fds[0].events = POLLOUT;         
        ret = poll(fds, 1, 10000); 
        assert(ret == 1);
        assert(fds[0].revents == POLLOUT);
    }

    //没有数据可读时，错误测试
    ret = read(sock, buf, 16);
    assert(ret < 0);
    assert(errno == EAGAIN);

    ret = write(sock, "hello", 5);
    assert(ret == 5);
    
    //等待可读
    fds[0].fd = sock;
    fds[0].events = POLLIN;         
    ret = poll(fds, 1, 10000); 
    assert(ret == 1);
    assert(fds[0].revents == POLLIN);  

    //读
    ret = read(sock, buf, 16);

    //关闭连接
    close(sock);
}


int main(){
    int ret = 0;
    test_socket();
    ret +=  test_file_stdc();    
    //return test_file_posix(); 
    return ret;
}



