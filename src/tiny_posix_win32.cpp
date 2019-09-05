#ifdef _WIN32

#include <memory>
#include <unordered_map>

#include "tiny_posix.h"
/*
class FDDriver{
public:
    virtual VOID Close(HANDLE file)=0;
    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op)=0;
    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op)=0;
    virtual BOOL GetResult(HANDLE file, LPOVERLAPPED op, LPDWORD len)=0;
};

class FileDriver : public FDDriver{
public:
    virtual VOID Close(HANDLE file){
        CloseHandle(file);
    }

    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return ReadFile(file, buf, buflen, NULL, op);
    }

    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WriteFile(file, buf, buflen, NULL, op);
    }

    virtual BOOL GetResult(HANDLE file, LPOVERLAPPED op, LPDWORD len){
        return GetOverlappedResult(file, op, len, FALSE);
    }
};

class SocketDriver : public FDDriver{
public:    
    virtual VOID Close(HANDLE file){
        closesocket(file);
    }
    virtual BOOL GetResult(HANDLE file, LPOVERLAPPED op, LPDWORD len){
        return WSAGetOverlappedResult(file, op, len, FALSE);
    }
    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return -1;
    }
    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return -1;
    }
};

class TCPClientDriver : public SocketDriver{
public: 
    virtual BOOL Open(HANDLE file, PVOID addr, DWORD addrlen){
        
    }

    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSARecv(file, buf, buflen, NULL, op, NULL);
    }
    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSASend(file, buf, buflen, NULL, op, NULL);
    }    
};
class TCPServerDriver : public SocketDriver{
public: 
    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSARecv(file, buf, buflen, NULL, op, NULL);
    }
    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSASend(file, buf, buflen, NULL, op, NULL);
    }    
};
class UDPSocketDriver : public SocketDriver{
public: 
    virtual DWORD Read(HANDLE file, PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSARecv(file, buf, buflen, NULL, op, NULL);
    }
    virtual DWORD Write(HANDLE file, const PVOID buf, DWORD buflen, LPOVERLAPPED op){
        return WSASend(file, buf, buflen, NULL, op, NULL);
    }    
};
*/

class FileDescriptor{
public:
    virtual ~FileDescriptor(){};

    virtual size_t Read(void* buf, size_t len) = 0;

    virtual size_t Write(const void* buf, size_t len) = 0;

    virtual void Close() = 0;

    virtual int Control(int cmd, void* val) = 0;

    virtual off_t Seek(off_t pos, int where) = 0;

    //预检测，如果有事件，直接返回事件,如果没有，返回0
    virtual short PrePoll(short event) = 0;

    //事件等待后，调用这个
    virtual void PostPoll() = 0;

    //获取event handle，用以等待事件发生
    virtual HANDLE GetEvent() = 0;

    //获取file handle， 用以原生API操作
    virtual HANDLE GetHandle() = 0;
};


//常规文件
class GenericFileDescriptor{
public:
    GenericFileDescriptor(HANDLE file){

    }
    ~GenericFileDescriptor(){
        
    }
    HANDLE GetEvent(){
        return op_.hEvent;
    }
    HANDLE GetHandle(){
        return file_;
    }
    size_t Read(void* buf, size_t len){
        if(nonblock_){
            if(readed_){ //非阻塞，已读到结果
                if(readed_ > len){
                    memcpy(buf, buf_, len);
                    memmove(buf_, buf_ + len, readed_ - len);
                    readed_ -= len;                    
                }else{
                    memcpy(buf, buf_, readed_);
                    len = readed_;
                    readed_ = 0;                    
                } 
                return len;             
            }else{ //非阻塞，未读到结果
                BOOL ret = ReadFile(file_, buf_, buflen_, NULL, &op_);
                return -1;
            }
        }else{ //阻塞方式读
            DWORD rdlen = 0;
            BOOL bret = ReadFile(file_, buf, len, &rdlen, NULL);
            return rdlen;
        }
    }

    size_t Write(const void* buf, size_t len){
        //任何时候都是阻塞方式写
        DWORD wrlen = 0;
        BOOL bret = WriteFile(file_, buf, len, &wrlen, NULL);
        return wrlen;
    }

    virtual void Close(){
        CloseHandle(file_);
    }
    virtual int Control(int cmd, void* val){
        return -1;
    }
    virtual off_t Seek(off_t pos, int where){
        return -1;
    }

    virtual short PrePoll(short event, HANDLE* outhandle){
        if(!nonblock_)return event;
        if(readed_)return POLLIN;
        if(event & POLLOUT)return POLLOUT;
        *outhandle = op_.hEvent;
        return 0;
    }
    
    virtual void PostPoll(){        
        BOOL bret = GetOverlappedResult(file_, &op_, &readed_, FALSE);
    }  

private:    
    HANDLE file_;   
    OVERLAPPED op_;
    CHAR* buf_;
    DWORD buflen_;
    DWORD readed_;
    bool nonblock_;
};

class SocketFileDescriptor{
public:
    SocketFileDescriptor(SOCKET sock){

    }
    ~SocketFileDescriptor(){
        
    }
    virtual HANDLE GetHandle(){
        return (HANDLE)sock_;
    }

    size_t Read(void* buf, size_t len){
        return recv(sock_, buf, len, 0);
    }

    size_t Write(const void* buf, size_t len){
        return send(sock_, buf, len, 0);
    }

    virtual void Close(){
        closesocket(sock_);
    }
    virtual int Control(int cmd, void* val){
        return -1;
    }
    virtual off_t Seek(off_t pos, int where){
        return -1;
    }

    virtual short PrePoll(short event, HANDLE* outhandle){
       
        return 0;
    }
    
    virtual void PostPoll(){        
       // BOOL bret = GetOverlappedResult(file_, &op_, &readed_, FALSE);
    }  

private:    
    SOCKET sock_;   
    HANDLE event_; 
    short cur_events_;   
    bool nonblock_;
};


int posix_poll(struct posix_pollfd* pfds, unsigned int nfds, int timeout){
    
    return -1;
}


ssize_t posix_read(int fd, void * buf, size_t count){
    return 0;
}
ssize_t posix_write(int fd, const void* buf, size_t count){
    return 0;
}

int posix_close(int fd){
    return 0;
}
int posix_open(const char* pathname, int flags, ...){
    return -1;
}
int posix_fcntl(int fd, int cmd, ...){
    return 0;
}

#endif


