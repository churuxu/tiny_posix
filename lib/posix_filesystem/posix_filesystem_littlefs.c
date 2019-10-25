
#include <errno.h>
#include <string.h>
#include "posix_filesystem.h"
#include "storage_device.h"
#include "lfs.h"


static struct lfs_config cfg_;
static lfs_t lfs_;

//lfs错误码转标准错误码
static int lfs_error_to_errno(int err){
    switch (err){
        case LFS_ERR_OK:return 0;
        case LFS_ERR_INVAL:return EINVAL;
        case LFS_ERR_EXIST:return EEXIST;
        case LFS_ERR_NOENT:return ENOENT;
        case LFS_ERR_IO:return EIO;
        case LFS_ERR_NOTDIR:return ENOTDIR;
        case LFS_ERR_ISDIR:return EISDIR;
        case LFS_ERR_NOTEMPTY:return ENOTEMPTY;
        case LFS_ERR_FBIG:return EFBIG;        
        case LFS_ERR_NOSPC:return ENOSPC;
        case LFS_ERR_NOMEM:return ENOMEM;
        case LFS_ERR_NAMETOOLONG:return ENAMETOOLONG;
        default:return EIO;
    }
    return -1;
}

//打开文件
POSIX_FILE* posix_fopen(const char* name, const char* mode){
    int err = EINVAL;
    int flag = 0;
    int ret;
    lfs_file_t* fp = malloc(sizeof (lfs_file_t));
    if(!fp){err = ENOMEM; goto error;}

    while (*mode){
        switch (*mode){
            case 'r': flag |= ( LFS_O_RDONLY ); break;
            case 'w': flag |= ( LFS_O_CREAT | LFS_O_WRONLY | LFS_O_TRUNC ); break;
            case 'a': flag |= ( LFS_O_APPEND | LFS_O_WRONLY ); break;
            case '+': flag |= ( LFS_O_CREAT |LFS_O_RDWR ); break;
            default: break;
        }
        mode ++;
    }
    if (!flag )flag = LFS_O_RDONLY;
    
    ret = lfs_file_open(&lfs_, fp, name, flag);
    if(ret){
        err = lfs_error_to_errno(ret);
        goto error;
    }
    return (POSIX_FILE*)fp;
error:
    if(fp)free(fp);
    errno = err;
    return NULL;
}

//关闭文件
int posix_fclose(POSIX_FILE* file){  
    int ret = EINVAL;  
    if(file){
        ret = lfs_file_close(&lfs_, (lfs_file_t*)file);
        free(file);
        ret = 0;
    }
    return ret;
}

//判断文件读写指针是否位于文件末尾
int posix_feof(POSIX_FILE* file){
    lfs_file_t* fp = (lfs_file_t*)file;
    lfs_soff_t sz = lfs_file_size(&lfs_, fp);
    lfs_soff_t cur = lfs_file_tell(&lfs_, fp);
    return cur >= sz;
}

//文件buffer中未写入完的数据写入存储
int posix_fflush(POSIX_FILE* file){
    lfs_file_t* fp = (lfs_file_t*)file;
    int ret = lfs_file_sync(&lfs_, fp);
    return ret;
}

//读文件
size_t posix_fread(void* buffer, size_t nitem, size_t length, POSIX_FILE* file){
    lfs_file_t* fp = (lfs_file_t*)file;
    int ret = lfs_file_read(&lfs_, fp, buffer, nitem * length);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
    }
    return ret;
}

//写文件
size_t posix_fwrite(const void* data, size_t nitem, size_t length, POSIX_FILE* file){
    lfs_file_t* fp = (lfs_file_t*)file;
    int ret = lfs_file_write(&lfs_, fp, data, nitem * length);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
    }
    return ret;    
}

//设置读写指针位置
int posix_fseek(POSIX_FILE* file, long offset, int origin){
    lfs_file_t* fp = (lfs_file_t*)file;
    int ret = lfs_file_seek(&lfs_, fp, offset,  origin);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
        return ret;
    }
    return 0;   
}

//获取读写指针位置
long posix_ftell(POSIX_FILE* file){
    lfs_file_t* fp = (lfs_file_t*)file;
    return lfs_file_tell(&lfs_, fp);
}





