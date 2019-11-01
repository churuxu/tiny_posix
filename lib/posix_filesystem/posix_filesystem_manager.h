#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//#include "fs_interface.h"
#include "storage_interface.h"

#ifdef __cplusplus
extern "C" {
#endif


int fs_manager_mount(const char* path, const char* fsname, storage_interface* stg);

int fs_manager_unmount(storage_interface* stg);

int fs_manager_mkfs(storage_interface* stg, const char* fsname);


/*
相对路径，第一个存储设备
绝对路径，可移动存储设备


插入SD时
  挂载到/sdcard
拔出SD时，卸载

插入U盘时(u盘可能多个)
  挂载到/usb0 /usb1 /usb2
拔出U盘时，卸载


*/

#ifdef __cplusplus
}
#endif


