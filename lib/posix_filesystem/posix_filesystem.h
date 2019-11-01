#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef __GNUC__
#include <unistd.h>
#include <sys/stat.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <sys/statvfs.h>
#endif



#ifdef __cplusplus
extern "C" {
#endif


/*
文件管理 读写  删除 截断 获取大小等
目录管理 建立 删除 遍历
*/


// =================== io ======================

typedef struct POSIX_FILE POSIX_FILE;

POSIX_FILE* posix_fopen(const char* name, const char* mode);
int posix_fclose(POSIX_FILE* file);
int posix_feof(POSIX_FILE* file);
int posix_fflush(POSIX_FILE* file);
size_t posix_fread(void* data, size_t nitem, size_t length, POSIX_FILE* file);
size_t posix_fwrite(const void* data, size_t nitem, size_t length, POSIX_FILE* file);
int posix_fseek(POSIX_FILE* file, long offset, int origin);
long posix_ftell(POSIX_FILE* file);

// =================== file ======================

struct posix_stat{
    off_t st_size; //文件大小
    int st_mode;  //文件类型 和 文件选项
};

int posix_access(const char* name, int mode);
int posix_remove(const char* filename);
int posix_rename(const char* oldname, const char* newname);
int posix_truncate(const char* name, off_t len);
int posix_stat(const char* name, struct posix_stat* info);


//=================== dir ======================
typedef struct POSIX_DIR POSIX_DIR;

struct posix_dirent{
    char* d_name;
};

POSIX_DIR* posix_opendir(const char* name);
int posix_closedir(POSIX_DIR* dir);
struct posix_dirent* posix_readdir(POSIX_DIR* dir);

int posix_rmdir(const char* name);
int posix_mkdir(const char* name, int mode);


//=================== disk ======================

struct posix_statvfs{    
    uint64_t f_blocks; //总块数量
    uint64_t f_bavail; //可用块数量
    uint32_t f_frsize; //块大小
};

int posix_statvfs(const char* path, struct posix_statvfs* info);



//=================== 用户使用宏 ======================
#ifndef POSIX_FILESYSTEM_IMPL

#define FILE POSIX_FILE
#define fopen posix_fopen
#define fclose posix_fclose
#define fwrite posix_fwrite
#define fread posix_fread
#define fseek posix_fseek
#define ftell posix_ftell
#define feof posix_feof

#define remove posix_remove
#define rename posix_rename
#define access posix_access
#define truncate posix_truncate
#define stat posix_stat

#define DIR POSIX_DIR 
#define dirent posix_dirent 
#define opendir posix_opendir
#define closedir posix_closedir
#define readdir posix_readdir

#define rmdir posix_rmdir
#define mkdir posix_mkdir

#define statvfs posix_statvfs

#endif

#ifdef __cplusplus
}
#endif


