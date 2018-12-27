#define RUNTIME_IMPL
#define pollfd pollfd_win32

#include "posix_runtime.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/stat.h>
#endif
#include <unordered_map>


#define CONNECTION_TYPE_TCP SOCK_STREAM
#define CONNECTION_TYPE_UDP SOCK_DGRAM

#ifdef _MSC_VER
#pragma warning(disable:4267)
#pragma comment(lib,"ws2_32.lib")
#endif

#define UNUSED(expr)  (void)(expr)

#ifdef _WIN32
typedef SOCKET socket_t;
#define sock_errno WSAGetLastError()
//test error code
#define io_is_in_progress(code) ((code) == WSAEINPROGRESS || (code) == WSAEWOULDBLOCK)
#define io_is_would_block(code) ((code) == WSAEWOULDBLOCK || (code) == ERROR_IO_PENDING) 

#define is_valid_file(fd) (fd)
#define is_valid_socket(fd) (fd!=INVALID_SOCKET)

#else
typedef int socket_t;
#define sock_errno errno
#define io_is_in_progress(code) ((code) == EINPROGRESS)
#define io_is_would_block(code) ((code) == EWOULDBLOCK) 

#define closesocket close

#define is_valid_file(fd) (fd>=0)
#define is_valid_socket(fd) (fd>=0)
#define INVALID_SOCKET -1
#endif

#ifdef _WIN32
#define mssleep Sleep
int io_set_nonblocking(socket_t fd){
	u_long nblock = 1;
	int ret = ioctlsocket(fd, FIONBIO, &nblock);
	if (ret)return WSAGetLastError();
	return 0;
}

#else

void mssleep(int ms){
    sleep(ms / 1000);
    usleep((ms%1000)*1000);
}

int io_set_nonblocking(socket_t fd) {
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0) return -1;
	if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)return -1;
	return 0;
}

#endif

#define PACKET_LOG(...) printf(__VA_ARGS__)
#define POLL_LOG(...) printf(__VA_ARGS__)


void print_bytes(const void* data, int len){    
    uint8_t* ptr = (uint8_t* )data; 
    UNUSED(ptr);   
    for(int i=0; i<len; i++){
        PACKET_LOG("%02X ", ptr[i]);    
    }
    
}


typedef struct fd_handle{
    fd_handle(){
        fdid = 0;
        file = NULL;        
        userdata = NULL;
        addrlen = 0;
        connected = 0;
        type = 0;
        disconnected = 0;
        memset(&sock, 0 ,sizeof(sock));
        memset(&ss, 0 ,sizeof(ss));
    }
    int fdid;
    FILE* file;   
    void* userdata;
    socket_t sock;
    socklen_t addrlen;
    int type;
    int connected;
    int disconnected;
    struct sockaddr_storage ss;
}fd_handle;

static std::unordered_map<int, fd_handle*> fdtable_;
static int fdid_;




int close_win32(int fd){
    fd_handle* handle = fdtable_[fd];
    if(handle->file){
        fclose(handle->file);
    }else{
        closesocket(handle->sock);
    }    
    free(handle);
    fdtable_.erase(fd);
    return 0;
}


int flash_open(){
    const char* filename = "flash.dat";
    FILE* f = fopen(filename, "rwb");
    if(!f){
        //create 64k file
        f = fopen(filename, "wb");
        int buflen = 1024*64;
        void* buf = malloc(buflen);
        memset(buf, 0, buflen);
        fwrite(buf,1,buflen, f);
        fclose(f);
    }
    f = fopen(filename, "rwb");
    if(!f)return -1;
    fd_handle* handle = new fd_handle();
    handle->file = f;
    fdid_ ++;
    handle->fdid = fdid_;
    fdtable_[fdid_] = handle;
    return fdid_;
}

int port_open(int num){
    return -1;
}

int socket_win32(int af, int t, int p){
    fd_handle* handle = new fd_handle();
    socket_t sock = socket(af, t, 0);
    if(!is_valid_socket(sock))return -1;
    handle->type = t;
    handle->sock = sock;
    fdid_ ++;
    handle->fdid = fdid_;
    handle->addrlen = 0;
    fdtable_[fdid_] = handle;
    return fdid_;     
}


int connect_win32(int fd, void* addr, int len){
    fd_handle* handle = fdtable_[fd];
    if(handle->type == CONNECTION_TYPE_UDP){  
        handle->addrlen  = len;
        memcpy(&handle->ss,addr,len);
        return 0;
    }else if(handle->type == CONNECTION_TYPE_TCP){
        return connect(handle->sock, (struct sockaddr*)addr, len);
    } 
    return -1;   
}


int fcntl_win32(int fd, int type, int v){
    if(type == F_SETFL){
        if(v & O_NONBLOCK){
            fd_handle* handle = fdtable_[fd];
            io_set_nonblocking(handle->sock);
        }
        return 0;
    }else{
        return 0;
    }
}

int errno_win32(){
    int e = WSAGetLastError();
    switch(e){
        case WSAEWOULDBLOCK:return EWOULDBLOCK;
        //case WSAE//:return EWOULDBLOCK;
    }
    return errno;
}


int socket_open(int type, void* addrptr, int addrlen){
    struct sockaddr* addr = (struct sockaddr* )addrptr;
    socket_t sock;
    fd_handle* handle = new fd_handle();
    if(type == CONNECTION_TYPE_UDP){        
        sock = socket(addr->sa_family, SOCK_DGRAM, 0);
        io_set_nonblocking(sock);
        //handle->connected = 1;
    }else if(type == CONNECTION_TYPE_TCP){
        sock = socket(addr->sa_family, SOCK_STREAM, 0);
        io_set_nonblocking(sock);
        int ret = connect(sock, addr, addrlen);
        int err = sock_errno;
        if(ret < 0 && !io_is_would_block(err))return ret;
    }else{
        return -1;
    }
    
    handle->type = type;
    handle->sock = sock;
    handle->addrlen  = addrlen;
    memcpy(&handle->ss,addrptr,addrlen);
    fdid_ ++;
    handle->fdid = fdid_;
    fdtable_[fdid_] = handle;
    return fdid_;    
}


ssize_t read_win32(int fd, void* buf, size_t buflen){
    fd_handle* handle = fdtable_[fd];
	ssize_t ret = -1;
    if(!handle)return -1;
    if(handle->file){ //is file
        ret = (ssize_t)fread(buf, 1, buflen, handle->file);
    }else{ //is socket
        if(handle->type != CONNECTION_TYPE_TCP){
            struct sockaddr_storage ss;
            socklen_t len = sizeof(ss);
            ret = recvfrom(handle->sock, (char*)buf, buflen, 0, (struct sockaddr*)&ss, &len);
        }else{
            ret = recv(handle->sock, (char*)buf, buflen, 0);
        }
        if(ret <0 && io_is_would_block(sock_errno))return -1;        
    }
    if(ret>=0){
        PACKET_LOG("fd: $%d readed %d bytes:\n", fd, (int)ret);
        print_bytes(buf, ret);
        PACKET_LOG("\n");
    }else{
        handle->disconnected = 1;
        printf("fd: $%d read error: %d\n", fd, sock_errno);
    }
    return ret;
}


ssize_t write_win32(int fd, const void* buf, size_t buflen){
    fd_handle* handle = fdtable_[fd];
    if(!handle)return -1;
    ssize_t ret = -1;
    if(handle->file){ //is file
        ret = fwrite(buf, 1, buflen, handle->file);
    }else{ //is socket
        if(handle->type != CONNECTION_TYPE_TCP){
            ret = sendto(handle->sock, (char*)buf, buflen, 0, (struct sockaddr*)&handle->ss, handle->addrlen);
        }else{
            ret = send(handle->sock, (char*)buf, buflen, 0);
        }
        if(ret <0 && io_is_would_block(sock_errno))ret = buflen;        
    }
    if(ret>=0){
        PACKET_LOG("fd: $%d writed %d bytes:\n", fd, (int)ret);
        print_bytes(buf, ret);
        PACKET_LOG("\n");
    }else{
        handle->disconnected = 1;
        printf("fd: $%d write error: %d\n", fd, sock_errno);
    }
    return ret;   
}

off_t lseek(int fd, off_t offset, int where){
    fd_handle* handle = fdtable_[fd];
    if(!handle)return -1;
    if(handle->file){ //is file
        return fseek(handle->file, offset, where);
    }
    return 0;    
}
#ifdef _WIN32
class Win32Initer{
public:
    Win32Initer(){
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2),&wsa);  
    }
};
Win32Initer init_;
#endif 

int runtime_init(){
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2),&wsa);
#endif

    struct addrinfo * ai = NULL;
    getaddrinfo("127.0.0.1","8000",NULL,&ai);
    socklen_t addrlen = ai->ai_addrlen;
    uint8_t buf[16];
    memcpy(buf, ai->ai_addr, addrlen);
    freeaddrinfo(ai);
    return 0;
}


int poll_win32(struct pollfd* fds, unsigned int nfds, int timeoutms){
    fd_set rfds;
    fd_set wfds;
    fd_set efds;
    int fdcount = 0;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_ZERO(&efds);
    int ret = 0;
	unsigned int i;
    std::unordered_map<int,fd_handle*> fdtable = fdtable_;
    uint64_t t0 = clock();
    UNUSED(t0);
    for(i=0;i<nfds;i++){
        int fd = fds[i].fd;
        short events = fds[i].events;
        fd_handle* handle = fdtable[fd];
        if(handle && handle->sock){
            if(events & POLLIN){
                FD_SET(handle->sock,&rfds); 
            }
            if(events & POLLOUT){
                if(handle->type == CONNECTION_TYPE_UDP){ //UDP直接可写
                    fds[i].revents |= POLLOUT;
                    ret++;
                }else{
                    FD_SET(handle->sock,&wfds); 
                }
            }
            FD_SET(handle->sock,&efds);
            fdcount ++; 
        }
    }
    if(!fdcount)return 0;
    if(ret)return ret;
    struct timeval timeout;
    struct timeval* ptimeout;
    if(timeoutms<0){
        ptimeout = NULL;
    }else{
        timeout.tv_sec = timeoutms / 1000;
        timeout.tv_usec = (timeoutms % 1000) * 1000;
        ptimeout = &timeout;
    }

    POLL_LOG("poll fds: ");
    for(i=0;i<nfds;i++){
        POLL_LOG("$%d ",fds[i].fd);
    }
    POLL_LOG(" %d ms ...",timeoutms);

    ret = select(fdcount, &rfds, &wfds, &efds, ptimeout);
    uint64_t t1 = clock();
    UNUSED(t1);
    if(ret<=0){
        POLL_LOG(" used %d ms (no event)\n", (int)(t1-t0));
        return 0;
    }else{
        POLL_LOG(" used %d ms ", (int)(t1-t0));
    }
    ret = 0;   
    for(i=0;i<nfds;i++){
        int fd = fds[i].fd;
        //short events = fds[i].events;
        fd_handle* handle = fdtable[fd];
        fds[i].revents = 0;
        if(handle && handle->sock){
            if(FD_ISSET(handle->sock, &rfds)){
                POLL_LOG(" $%d:POLLIN", fd);
                fds[i].revents |= POLLIN;
            }
            if(FD_ISSET(handle->sock, &wfds)){
                POLL_LOG(" $%d:POLLOUT", fd);
                fds[i].revents |= POLLOUT;
            }
            if(FD_ISSET(handle->sock, &efds)){
                POLL_LOG(" $%d:POLLERR", fd);
                fds[i].revents |= POLLERR;
            }
            ret++;
        }
    }
    POLL_LOG("\n");
    return ret;
}

