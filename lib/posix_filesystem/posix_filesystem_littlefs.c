#define POSIX_FILESYSTEM_IMPL
#include <errno.h>
#include <string.h>
#include "posix_filesystem.h"
#include "posix_filesystem_manager.h"
#include "lfs.h"


//一个lfs文件系统上下文  (用到的所有变量)
typedef struct lfs_context{
    int mounted;
    lfs_t lfs;
    struct lfs_config cfg;
}lfs_context;

//文件
struct POSIX_FILE{
    lfs_t* plfs;
    lfs_file_t file;
};

//目录
struct POSIX_DIR{
    lfs_t* plfs;
    lfs_dir_t dir;
    struct lfs_info info;
    struct posix_dirent ent;
};

//全局唯一文件系统，以后可能改成多个
static lfs_context fs_;

//分配一个文件系统， （目前只支持一个文件系统，以后可能改成多个）
static lfs_context* alloc_lfs_context(){
    if(!fs_.mounted){
        return &fs_;
    } 
    return NULL;   
}

//获取指定路径挂载的文件系统 （目前只支持一个文件系统，以后可能改成多个）
static lfs_context* get_lfs_context(const char* path){
    if(fs_.mounted){
        return &fs_;
    }
    errno = ENODEV;
    return NULL;
}

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
    int flag = 0;
    int ret;
    POSIX_FILE* fp = NULL;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return NULL;}

    fp = (POSIX_FILE* ) malloc(sizeof (POSIX_FILE));
    if(!fp){errno = ENOMEM; return NULL;}

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
    
    ret = lfs_file_open(&fs->lfs, &fp->file, name, flag);
    if(ret){
        errno = lfs_error_to_errno(ret);
        free(fp);
        return NULL;
    }
    fp->plfs = &fs->lfs;
    return (POSIX_FILE*)fp;
}

//关闭文件
int posix_fclose(POSIX_FILE* fp){  
    int ret = EINVAL;  
    if(fp){
        ret = lfs_file_close(fp->plfs, &fp->file);
        free(fp);
        ret = 0;
    }
    return ret;
}

//判断文件读写指针是否位于文件末尾
int posix_feof(POSIX_FILE* fp){    
    lfs_soff_t sz = lfs_file_size(fp->plfs, &fp->file);
    lfs_soff_t cur = lfs_file_tell(fp->plfs, &fp->file);    
    return cur >= sz;
}

//文件buffer中未写入完的数据写入存储
int posix_fflush(POSIX_FILE* fp){    
    int ret = lfs_file_sync(fp->plfs, &fp->file);
    return ret;
}

//读文件
size_t posix_fread(void* buffer, size_t nitem, size_t length, POSIX_FILE* fp){    
    int ret = lfs_file_read(fp->plfs, &fp->file, buffer, nitem * length);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
    }
    return ret;
}

//写文件
size_t posix_fwrite(const void* data, size_t nitem, size_t length, POSIX_FILE* fp){    
    int ret = lfs_file_write(fp->plfs, &fp->file, data, nitem * length);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
    }
    return ret;    
}

//设置读写指针位置
int posix_fseek(POSIX_FILE* fp, long offset, int origin){    
    int ret = lfs_file_seek(fp->plfs, &fp->file, offset,  origin);
    if(ret<0){
        errno = lfs_error_to_errno(ret);
        return ret;
    }
    return 0;   
}

//获取读写指针位置
long posix_ftell(POSIX_FILE* fp){    
    return lfs_file_tell(fp->plfs, &fp->file);
}

//删除文件
int posix_remove(const char* filename){
    int ret;
    lfs_context* fs = get_lfs_context(filename);
    if(!fs){ return -1;}
    ret = lfs_remove(&fs->lfs, filename);
    if(ret){ errno = lfs_error_to_errno(ret); }
    return ret;
}

//重命名文件
int posix_rename(const char* oldname, const char* newname){
    int ret;
    lfs_context* newfs; 
    lfs_context* fs = get_lfs_context(oldname);
    if(!fs){ return -1;} 
    newfs = get_lfs_context(newname);
    if(!newfs || newfs != fs){errno = EXDEV; return -1;} //跨文件系统移动
    ret = lfs_rename(&fs->lfs, oldname, newname);
    if(ret){ errno = lfs_error_to_errno(ret); }
    return ret;
}

//判断文件是否可读写
int posix_access(const char* name, int mode){
    int ret;
    struct lfs_info info;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1;}    
    ret = lfs_stat(&fs->lfs, name, &info);
    if(ret){ errno = lfs_error_to_errno(ret); return -1; }
    if(info.type == LFS_TYPE_REG)return 0;
    return -1;    
}

//截断文件
int posix_truncate(const char* name, off_t len){
    int ret;
    lfs_file_t f;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1;} 
    ret = lfs_file_open(&fs->lfs, &f, name, LFS_O_RDWR);
    if(ret){ errno = lfs_error_to_errno(ret); return -1; }    
    ret = lfs_file_truncate(&fs->lfs, &f, len);
    lfs_file_close(&fs->lfs, &f);
    if(ret){ errno = lfs_error_to_errno(ret); return -1; }
    return ret;    
}

//获取文件属性
int posix_stat(const char* name, struct posix_stat* info){
    int ret;
    struct lfs_info lfsinfo;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1;} 
    ret = lfs_stat(&fs->lfs, name, &lfsinfo);
    if(ret){ errno = lfs_error_to_errno(ret); return -1; }
    if(lfsinfo.type == LFS_TYPE_REG){
        info->st_mode = S_IFREG;
        info->st_size = lfsinfo.size;
        return 0;
    }else if(lfsinfo.type == LFS_TYPE_DIR){
        info->st_mode = S_IFDIR;
        info->st_size = 0;
        return 0;
    }
    info->st_mode = 0;
    info->st_size = 0;
    return -1;     
}

//删除目录
int posix_rmdir(const char* name){
    int ret;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1;} 
    ret = lfs_remove(&fs->lfs, name);
    if(ret){ errno = lfs_error_to_errno(ret); }
    return ret;    
}

//创建目录
int posix_mkdir(const char* name, int mode){
    int ret;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1;} 
    ret = lfs_mkdir(&fs->lfs, name);
    if(ret){ errno = lfs_error_to_errno(ret); }
    return ret;    
}

//打开目录
POSIX_DIR* posix_opendir(const char* name){
    int ret;    
    POSIX_DIR* fp = NULL;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return NULL;}
    fp = (POSIX_DIR* )malloc(sizeof (POSIX_DIR));
    if(!fp){ errno = ENOMEM; return NULL;}     
    ret = lfs_dir_open(&fs->lfs, &fp->dir, name);
    if(ret){ errno = lfs_error_to_errno(ret); free(fp); return NULL;}
    fp->plfs = &fs->lfs;
    return fp;
}

//关闭目录
int posix_closedir(POSIX_DIR* fp){
    int ret = EINVAL;  
    if(fp){
        ret = lfs_dir_close(fp->plfs, &fp->dir);
        free(fp);
        ret = 0;
    }
    return ret;    
}

//遍历目录
struct posix_dirent* posix_readdir(POSIX_DIR* fp){
    int ret;
    ret = lfs_dir_read(fp->plfs, &fp->dir, &fp->info);
    if(ret){ errno = lfs_error_to_errno(ret); return NULL;}
    fp->ent.d_name = fp->info.name;
    return &fp->ent;    
}

//获取存储设备的总大小，已用大小
int posix_statvfs(const char* name, struct posix_statvfs* info){    
    lfs_ssize_t sz;
    lfs_context* fs = get_lfs_context(name);
    if(!fs){ return -1; }
    sz = lfs_fs_size(&fs->lfs);
    if(sz<0){ return -1; }
    info->f_frsize = fs->cfg.block_size;
    info->f_blocks = fs->cfg.block_count;
    info->f_bavail = fs->cfg.block_count - sz;
    return 0;     
}



//======================= driver layer ======================

static int lfs_read_op(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size){
    storage_interface* dev = c->context;
    int ret = dev->read_op(dev, block, off, buffer, size); 
    switch(ret){
        case 0:return 0;
        case STORAGE_DEVICE_ERROR_IO: return LFS_ERR_IO;
        case STORAGE_DEVICE_CORRUPT: return LFS_ERR_CORRUPT;
        default:return LFS_ERR_IO;
    }    
}

static int lfs_prog_op(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size){
    storage_interface* dev = c->context;  
    int ret = dev->write_op(dev, block, off, buffer, size); 
    switch(ret){
        case 0:return 0;
        case STORAGE_DEVICE_ERROR_IO: return LFS_ERR_IO;
        case STORAGE_DEVICE_CORRUPT: return LFS_ERR_CORRUPT;
        case STORAGE_DEVICE_READONLY: return LFS_ERR_INVAL;
        default:return LFS_ERR_IO;
    }     
}


static int lfs_erase_op(const struct lfs_config *c, lfs_block_t block){
    storage_interface* dev = c->context;
    int ret;
    if(!dev->erase_op)return 0;
    ret = dev->erase_op(dev, block); 
    switch(ret){
        case 0:return 0;
        case STORAGE_DEVICE_ERROR_IO: return LFS_ERR_IO;
        case STORAGE_DEVICE_CORRUPT: return LFS_ERR_CORRUPT;
        case STORAGE_DEVICE_READONLY: return LFS_ERR_INVAL;
        default:return LFS_ERR_IO;
    }    
}

static int lfs_sync_op(const struct lfs_config *c){
    return 0;
}


//设备驱动注册
static int storage_device_register(const storage_interface* dev, int mk){
    int ret;
    struct lfs_config* cfg;
    lfs_context* fs = alloc_lfs_context();
    if(!fs)return -1;
    cfg = &fs->cfg;

    cfg->context = (void*)dev;
    cfg->read = lfs_read_op;
    cfg->prog = lfs_prog_op;
    cfg->erase = lfs_erase_op;
    cfg->sync = lfs_sync_op;
    cfg->block_size = dev->block_size;
    cfg->block_count = dev->block_count;
    cfg->read_size = dev->rw_size;
    cfg->prog_size = dev->rw_size;
    cfg->block_cycles = 128;
    cfg->cache_size = dev->rw_size * 4;
    cfg->lookahead_size = dev->rw_size * 8;
    if(!mk){
        ret = lfs_mount(&fs->lfs, cfg);
        if(!ret){
            fs->mounted = 1;
        }
    }else{
        ret = lfs_format(&fs->lfs, cfg);   
    }    

    return ret;
}



int fs_manager_mount(const char* path, const char* fsname, storage_interface* stg){
    int ret;
    if(strcmp(fsname, "littlefs")){
        errno = EINVAL;
        return -1;
    }
    ret = storage_device_register(stg, 0);
    return ret;
}

int fs_manager_unmount(storage_interface* stg){
    return 0;
}

int fs_manager_mkfs(storage_interface* stg, const char* fsname){
    int ret = storage_device_register(stg, 1);
    return ret;    
}



