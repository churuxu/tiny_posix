#include "tiny_posix.h"



#define HOST "0.0.0.0"
#define PORT "9000"

#define MAX_FD 64


void example_tcp_server(){
    int fd, ret, i, len;
    char buf[1025];
    struct pollfd fds[MAX_FD];
    struct addrinfo* ai;   
    int serverfd, count, lost, haslost, newcount;

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

    ret = bind(fd, ai->ai_addr, ai->ai_addrlen);
    if(ret){       
        perror("bind socket error\n");
        return;
    }

    ret = listen(fd, 512);
    if(ret){       
        perror("listen socket error\n");
        return;
    }

    printf("listen on port %s ok\n", PORT);
    serverfd = fd;
    newcount = 1;
    fds[0].fd = fd;
    fds[0].events = POLLIN;
    fds[0].revents = 0;    
    while(1){
        count = newcount;
        ret = poll(fds, count, 10000);
        if(ret>0){
            newcount = count;
            haslost = 0;
            for(i=0;i<count;i++){
                fd = fds[i].fd;
                if(fd == serverfd){
                    if(!(fds[i].revents & POLLIN))continue;
                    ret = accept(serverfd, NULL, NULL);
                    if(ret<=0){
                        perror("accept error\n");
                    }
                    fds[newcount].fd = ret;
                    fds[newcount].events = POLLIN;
                    newcount ++;
                    printf("get a connection\n");
                }else{
                    lost = 0;
                    if(fds[i].revents & POLLERR){
                        lost = 1;
                    }
                    while(1){
                        ret = read(fd, buf, 1024);
                        if(ret>0){
                            buf[ret] = 0;
                            printf("recv %d bytes\n", ret);
                            len = write(fd, buf, ret);
                            if(ret!=len){
                                printf("write error\n");  
                                lost = 1;
                                break;
                            }
                        }else if(ret==0){                
                            //printf("remote close\n");
                            lost = 1;
                            break;
                        }else{
                            if(errno != EAGAIN && errno != EWOULDBLOCK){
                                printf("recv error\n");
                                lost = 1;
                            }
                            break;              
                        }
                    }
                    if(lost){
                        haslost = 1;
                        fds[i].fd = -1;
                        printf("lost a connection\n");
                    }
                } //server or client
            } //for fd

            //移除所有已失效的fd
            if(haslost){
                i=1;            
                while(1){
                    if(fds[i].fd < 0){
                        fds[i].fd = fds[newcount-1].fd;
                        newcount --;
                        if(i >= newcount)break;
                    }else{
                        i ++;
                        if(i>=newcount)break;
                    }
                }
            }
        } //ret > 0       
    }
}



