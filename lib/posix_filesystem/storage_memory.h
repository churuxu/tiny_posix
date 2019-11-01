#pragma once

#include "storage_interface.h"

#ifdef __cplusplus
extern "C" {
#endif


/* 
存储设备驱动示例：
使用内存模拟存储设备
*/


//初始化
storage_interface* storage_memory_create();


#ifdef __cplusplus
}
#endif
