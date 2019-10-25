#pragma once

/*
存储设备驱动接口

信息：
设备名称
块大小
块个数
单次读/写长度

初始化
写（第几块，偏移，数据，长度）
读（第几块，偏移，数据，长度）
擦除 (第几块)


*/

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif


//设备操作函数错误码
#define STORAGE_DEVICE_ERROR_IO      -100  //设备异常
#define STORAGE_DEVICE_READONLY  -101  //设备只读
#define STORAGE_DEVICE_CORRUPT   -102  //设备坏块


//存储设备驱动接口
typedef struct storage_device storage_device;

struct storage_device{
    //块大小
    size_t block_size; 

    //块数量
    size_t block_count; 

    //单次读写长度
    size_t rw_size; 

    //读一个块中的一小段数据
    //成功返回0， 失败返回错误码
    int (*read_op)(storage_device* device, size_t block, size_t off, void* buf, size_t len);

    //写一个块中的一小段数据
    //成功返回0， 失败返回错误码
    int (*write_op)(storage_device* device, size_t block, size_t off, const void* buf, size_t len);

    //擦除一块，返回
    //成功返回0， 失败返回错误码 
    int (*erase_op)(storage_device* device, size_t block);

};



//添加存储设备
int storage_device_mount(const storage_device* dev);



/*
以后可能扩展场景：
插入了U盘A，挂载
/  /usb0
插入了U盘B，挂载
/  /usb0  /usb1
插入了U盘C，挂载
/  /usb0  /usb1  /usb2
拔掉U盘B，卸载
/  /usb0  /usb2


代码已支持文件系统littlefs fatfs
插入的设备文件系统为f



*/


#ifdef __cplusplus
}
#endif