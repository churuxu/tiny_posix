#include "tiny_posix.h"


#define HOST "www.baidu.com"
#define PORT "80"
//#define DATA "GET / HTTP/1.1\r\nHost: www.baidu.com\r\nConnection: close\r\n\r\n"
#define DATA "GET / HTTP/1.1\r\nHost: www.baidu.com\r\n\r\n"


void example_tcp_client(){
    int fd, ret;
    char buf[1025];
    struct pollfd fds[1];
    struct addrinfo* ai;
    const char* data = DATA;
    int datalen = strlen(data);

    ret = getaddrinfo(HOST,PORT,NULL,&ai);
    if(ret){
        printf("getaddrinfo error\n");
        return;
    }
    fd = socket(ai->ai_family, SOCK_STREAM, 0);
    if(fd<0){
        printf("create socket error\n");
        return;
    }

    ret = fcntl(fd, F_GETFL, 0);
    ret = fcntl(fd, F_SETFL, ret|O_NONBLOCK);
    if(ret){
        printf("fcntl error\n");
        return;
    }

    ret = connect(fd, ai->ai_addr, ai->ai_addrlen);
    if(ret == 0){
        printf("connect ok immediately\n");
    }else if(errno != EWOULDBLOCK){
        printf("connect error immediately\n");
        return;
    }else{
        fds[0].fd = fd;
        fds[0].events = POLLOUT;
        fds[0].revents = 0;
        ret = poll(fds, 1, 5000);
        if(fds[0].revents & POLLOUT){
            printf("connect ok\n");
        }else{
            printf("connect error\n"); 
        }
    }

    ret = write(fd, data, datalen);
    if(ret != datalen){
        printf("socket write error\n");
        return;
    }

    fds[0].fd = fd;
    fds[0].events = POLLIN;
    fds[0].revents = 0;    
    while(1){
        ret = poll(fds, 1, 10000);
        if(ret>0 && fds[0].revents & POLLIN){
            while(1){
                ret = read(fd, buf, 1024);
                if(ret>0){
                    buf[ret] = 0;
                    printf("%s", buf);
                }else if(ret==0){                
                    printf("remote close\n");
                    return;
                }else{
                    if(errno != EAGAIN && errno != EWOULDBLOCK){
                        printf("recv error\n");
                        return;
                    }else{
                        break;
                    }                    
                }
            }

        }else{
            printf("recv timeout\n");
            break;
        }        
    }
}



