#pragma once

#include "storage_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
存储设备驱动示例：
使用文件模拟存储设备
*/


//初始化
storage_interface* storage_file_create(const char* name);


#ifdef __cplusplus
}
#endif
