#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "storage_file.h"


#define BLOCK_SIZE  4096 
#define BLOCK_COUNT 64
#define RW_SIZE     32
#define MAX_NAME 64

//#define LOG printf
#define LOG(...) 

typedef struct storage_file{
    storage_interface stg;
    FILE* file;
    char name[MAX_NAME];
}storage_file;


// 从文件读数据
static int storage_file_read(storage_interface* device, size_t block, size_t off, void* buf, size_t len){
    FILE* file;
    storage_file* pstg = (storage_file*)device;
    size_t absoffset = block * BLOCK_SIZE + off;
    int ret = -1;
    int rlen;
    //file = fopen(pstg->name, "r+b");
    file = pstg->file;
    if(file){
        ret = fseek(file, absoffset, SEEK_SET);
        if(!ret){ 
            rlen = fread(buf, 1, len, file);
            ret = (rlen == len)?0:STORAGE_DEVICE_ERROR_IO;
        }
        //fclose(file);
    }
    LOG("storage read block:%d offset:%d length:%d result:%d\n",(int)block,(int)off,(int)len,ret);
    return 0;
}

// 写数据到文件
static int storage_file_write(storage_interface* device, size_t block, size_t off, const void* buf, size_t len){
    FILE* file;
    storage_file* pstg = (storage_file*)device;
    size_t absoffset = block * BLOCK_SIZE + off;
    int ret = -1;
    int rlen;
    //file = fopen(pstg->name, "r+b");
    file = pstg->file;
    if(file){
        ret = fseek(file, absoffset, SEEK_SET);
        if(!ret){ 
            rlen = fwrite(buf, 1, len, file);
            ret = (rlen == len)?0:STORAGE_DEVICE_ERROR_IO;
        }
        //fclose(file);
        fflush(file);
    }
    LOG("storage write block:%d offset:%d length:%d result:%d\n",(int)block,(int)off,(int)len,ret);
    return 0;   
}


storage_interface* storage_file_create(const char* name){
    char* buf = NULL; 
    FILE* file = NULL; 
    
    storage_file* p = NULL;
    int i,ret,len;    
    size_t sz = BLOCK_COUNT * BLOCK_SIZE;
    //打开已有文件
    file = fopen(name, "r+b");
    if(file){
        fseek(file, 0, SEEK_END);
        //文件存在，并且大小匹配, 直接注册
        if(ftell(file) == sz){
            goto reg;
        }
        fclose(file);
    }
    //创建新文件
    file = fopen(name, "wb");
    if(!file)goto error;
    buf = (char*)malloc(BLOCK_SIZE);
    if(!buf)goto error;
    memset(buf, 0xff, BLOCK_SIZE);
    for(i = 0; i < BLOCK_COUNT; i++){
        ret = fwrite(buf,1,BLOCK_SIZE,file);
        if(ret != BLOCK_SIZE)goto error;
    }
    fclose(file);
    free(buf);
    buf = NULL;
    LOG("storage create ok\n");
    file = fopen(name, "r+b");
    if(!file)goto error;
reg:
    
    p = (storage_file*)malloc(sizeof(storage_file)); 
    if(!p)goto error;
    len = strlen(name) + 1;
    if(len > MAX_NAME)goto error;
    p->file = file;
    //初始化结构体
    memset(&p->stg,0,sizeof(storage_interface));
    p->stg.block_count = BLOCK_COUNT;
    p->stg.block_size = BLOCK_SIZE;
    p->stg.rw_size = RW_SIZE;
    p->stg.read_op = storage_file_read;
    p->stg.write_op = storage_file_write;    
    memcpy(p->name, name, len); 

    return (storage_interface*)p;
error:
    if(file)fclose(file);
    if(buf)free(buf);
    if(p)free(p);
    return NULL;
}



