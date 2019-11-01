#include "storage_memory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define BLOCK_SIZE  4096
#define BLOCK_COUNT 64
#define RW_SIZE     32

//#define LOG printf
#define LOG(...) 


typedef struct storage_memory{
    storage_interface stg;
    uint8_t buf[BLOCK_SIZE * BLOCK_COUNT];
}storage_memory;



static int storage_memory_read(storage_interface* device, size_t block, size_t off, void* buf, size_t len){
    storage_memory* p = (storage_memory*)device;
    size_t absoffset = block * BLOCK_SIZE + off;
    memcpy(buf, &p->buf[absoffset], len);
    LOG("storage read block:%d offset:%d length:%d\n",(int)block,(int)off,(int)len);
    return 0;
}

static int storage_memory_write(storage_interface* device, size_t block, size_t off, const void* buf, size_t len){
    storage_memory* p = (storage_memory*)device;
    size_t absoffset = block * BLOCK_SIZE + off;
    memcpy(&p->buf[absoffset], buf, len);
    LOG("storage read block:%d offset:%d length:%d\n",(int)block,(int)off,(int)len);
    return 0;    
}

storage_interface* storage_memory_create(){
    storage_memory* p = NULL;

    p = (storage_memory*)malloc(sizeof(storage_memory));
    if(!p){
        LOG("storage alloc memory failed\n");
        return NULL;
    }

    memset(p->buf, 0xff, BLOCK_COUNT*BLOCK_SIZE);
    memset(&p->stg,0,sizeof(storage_interface));
    //初始化结构体
    p->stg.block_count = BLOCK_COUNT;
    p->stg.block_size = BLOCK_SIZE;
    p->stg.rw_size = RW_SIZE;
    p->stg.read_op = storage_memory_read;
    p->stg.write_op = storage_memory_write;
    
    return (storage_interface*)p;
}



