#include "gtest/gtest.h"
#include "posix_filesystem.h"
#include "posix_filesystem_manager.h"
#include "storage_memory.h"
#include "storage_file.h"

#define storage_create() storage_memory_create()
//#define storage_create() storage_file_create("storage.dat")


class posix_filesystem: public testing::Test {
public:
    static void SetUpTestCase() {
        storage_interface* stg = storage_create();
        ASSERT_TRUE(stg);
        int ret = fs_manager_mount(NULL, "littlefs", stg);
        if(ret){
            fs_manager_mkfs(stg, "littlefs");
            ret = fs_manager_mount(NULL, "littlefs", stg);
        }
        ASSERT_TRUE(!ret);
    }
};

#define MAXFILESIZE(X) (16+(X+5)%11)*1024;//max:26KB

/*
创建新文件，  多次字符串/二进制读写,写新文件,写覆盖老文件,一次写16KB以上
打开已有文件，存在则追加，不存在则创建,多次字符串/二进制读写,一次写16KB以上
*/
bool test_read_write_file(const char *filename,bool string_or_binary,bool append,uint32_t test_degree)
{
    uint32_t i,j,sizeoffile,offset;
    size_t sizeofwrite,sizeofread;

    int  ret;
    char string_write[]=" !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    char string_read[sizeof(string_write)]={0};
    char binary_write[1024]={0};
    char binary_read [1024]={0};
    FILE *file;
    for(i=0;i<test_degree;i++)
    {
        if(true==append)
        {
            if(true==string_or_binary)
            { 
                file=fopen(filename,"a+");
            }
            else
            {
                file=fopen(filename,"ab+");
            }
        }
        else
        {
            if(true==string_or_binary)
            { 
                file=fopen(filename,"w+");
            }
            else
            {
                file=fopen(filename,"wb+");
            }
        }
  
        if(NULL == file)
        {
            return false;
        }

        sizeoffile=MAXFILESIZE(test_degree);

        if(true==string_or_binary)
        {
            if(append)
            {
                fseek(file, 0L,SEEK_END); 
                offset=ftell(file);
            }
            //写入
            for(j=0;j<sizeoffile/(sizeof(string_write)-1);j++)
            {
                sizeofwrite=fwrite(string_write,1,sizeof(string_write)-1,file);
                if(sizeofwrite!=sizeof(string_write)-1) 
                    return false;      
            }
            if(append)
            {
                fseek(file,offset,SEEK_SET);
            }
            else
            {
                fseek(file,0,SEEK_SET);
            }
            //读取比较  
            for(j=0;j<sizeoffile/(sizeof(string_read)-1);j++)
            {
                memset(string_read,0,sizeof(string_read)-1);
                sizeofread=fread(string_read,1,sizeof(string_read)-1,file);
                if(sizeofread!=sizeof(string_read)-1) 
                    return false;    
                if(strcmp(string_read,string_write))
                    return false;  
            }
            
        }
        else
        {
            for(j=0;j<sizeof(binary_write);j++)
            {
                binary_write[j]=j;
            }

            if(append)
            {
                fseek(file, 0L,SEEK_END); 
                offset=ftell(file);
            }
            //写入
            for(j=0;j<sizeoffile/sizeof(binary_write);j++)
            {
                sizeofwrite=fwrite(binary_write,1,sizeof(binary_write),file);
                if(sizeofwrite!=sizeof(binary_write))
                    return false;
            }
            if(append)
            {
                fseek(file,offset,SEEK_SET);
            }
            else
            {
                fseek(file,0,SEEK_SET);
            }
            //读取比较
            for(j=0;j<sizeoffile/sizeof(binary_read);j++)
            {
                memset(binary_read,0,sizeof(binary_read));
                sizeofread=fread(binary_read,1,sizeof(binary_read),file);
                if(sizeofread!=sizeof(binary_read))
                    return false;
                if(memcmp(binary_read,binary_write,sizeof(binary_read)))
                    return false;
            }
        }
        ret=fclose(file);
        if(ret)
            return false;
    }
    if(remove(filename))
        return false;
    return true;
}

/*
读/写/读写/追加 fseek
读/写/读写/追加 ftell
fseek 负数
fseek 超出范围
*/
bool test_tell_seek_file(const char *filename,uint32_t test_degree)
{
    uint32_t i,sizeoffile,offset,offset_append;
    size_t sizeofwrite,sizeofread;

    char binary_write[1024+30]={0};
    char binary_read [1024]={0};
    FILE *file;
    for(i=0;i<sizeof(binary_write);i++)
    {
        binary_write[i]=255-i%256;
    }
    sizeoffile=MAXFILESIZE(test_degree);
    file=fopen(filename,"wb");
    if(NULL==file)
    {
        return false;
    }
    //写时seek,tell
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        sizeofwrite=fwrite(binary_write,1,sizeof(binary_write),file);
        if(sizeofwrite!=sizeof(binary_write))
            return false;
        offset=ftell(file);
        if(offset!=(i+1)*sizeof(binary_write)-i*(sizeof(binary_write)-sizeof(binary_read)))
            return false;
        if(fseek(file,(long)(sizeof(binary_read)-sizeof(binary_write)),SEEK_CUR))  //fseek SEEK_CUR 负数
            return false;
    }
    if(fclose(file))
        return false;
    
    file=fopen(filename,"rb");
    if(NULL==file)
    {
        return false;
    }
    //读时seek,tell
    if(fseek(file,(long)(sizeof(binary_read)-sizeof(binary_write)),SEEK_END))      //fseek SEEK_END 负数
        return false;
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        if(fseek(file,-(long)sizeof(binary_read),SEEK_CUR))
            return false;
        memset(binary_read,0,sizeof(binary_read));
        sizeofread=fread(binary_read,1,sizeof(binary_read),file);
        if(sizeofread!=sizeof(binary_read))
            return false;
        if(memcmp(binary_read,binary_write,sizeof(binary_read)))
            return false;
        if(fseek(file,-(long)sizeof(binary_read),SEEK_CUR))
            return false;
    }
    if(0!=ftell(file))
        return false;
    if(fseek(file,0,SEEK_END))                                      //fseek out of range
        return false;
    if(fseek(file,1,SEEK_CUR))
        return false;
    if(fread(binary_read,1,sizeof(binary_read),file))
        return false;
    if(fclose(file))
        return false;
    //读写时 seek,tell
    file=fopen(filename,"wb+");
    if(NULL==file)
    {
        return false;
    }
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        sizeofwrite=fwrite(binary_write,1,sizeof(binary_write),file);
        if(sizeofwrite!=sizeof(binary_write))
            return false;
        offset=ftell(file);
        if(offset!=(i+1)*sizeof(binary_write)-i*(sizeof(binary_write)-sizeof(binary_read)))
            return false;
        if(fseek(file,(long)(sizeof(binary_read)-sizeof(binary_write)),SEEK_CUR))  //fseek SEEK_CUR 负数
            return false;
    }
    if(fseek(file,(long)(sizeof(binary_read)-sizeof(binary_write)),SEEK_END))      //fseek SEEK_END 负数
        return false;
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        if(fseek(file,-(long)sizeof(binary_read),SEEK_CUR))
            return false;
        memset(binary_read,0,sizeof(binary_read));
        sizeofread=fread(binary_read,1,sizeof(binary_read),file);
        if(sizeofread!=sizeof(binary_read))
            return false;
        if(memcmp(binary_read,binary_write,sizeof(binary_read)))
            return false;
        if(fseek(file,-(long)sizeof(binary_read),SEEK_CUR))
            return false;
    }
    if(0!=ftell(file))
        return false;  
    if(fseek(file,0,SEEK_END))                                                //fseek out of range
        return false;
    if(fseek(file,1,SEEK_CUR))
        return false;
    if(fread(binary_read,1,sizeof(binary_read),file))
        return false;
    if(fclose(file))
        return false;
    //追加时 seek,tell
    file=fopen(filename,"ab+");
    if(NULL==file)
    {
        return false;
    }
    fseek(file,0,SEEK_END);
    offset_append=ftell(file);
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        sizeofwrite=fwrite(binary_write,1,sizeof(binary_write),file);
        if(sizeofwrite!=sizeof(binary_write))
            return false;
        offset=ftell(file);
        /*
        a+     
            Open for reading and appending (writing at end of file).  The file is created if it does not exist.  The initial file 
            position for  reading is at the beginning of the file, but output is always appended to the end of the file.
        */
        if(offset-offset_append!=(i+1)*sizeof(binary_write))
            return false;
        if(fseek(file,(long)(sizeof(binary_read)-sizeof(binary_write)),SEEK_CUR))  //fseek SEEK_CUR 负数
            return false;
    }
    if(fseek(file,0,SEEK_END))      
        return false;
    for(i=0;i<sizeoffile/sizeof(binary_read);i++)
    {
        if(fseek(file,-(long)sizeof(binary_write),SEEK_CUR))
            return false;
        memset(binary_read,0,sizeof(binary_read));
        sizeofread=fread(binary_read,1,sizeof(binary_read),file);
        if(sizeofread!=sizeof(binary_read))
            return false;
        if(memcmp(binary_read,binary_write,sizeof(binary_read)))
            return false;
        if(fseek(file,-(long)sizeof(binary_read),SEEK_CUR))
            return false;
    }
    uint32_t curoff = ftell(file);
    if(curoff!=offset_append)
        return false;
    if(fclose(file))
        return false;
    if(remove(filename))
        return false;
    return true;    
}

/*
删除不存在文件
重命名不存在文件
重命名目标文件已存在
读不存在文件
删除已打开文件
重命名已打开文件
多次打开同一文件读
打开文件读，再打开文件写
*/
bool test_remove_rname_file(const char *filename)
{    
    char name0[255]={0};
    char name1[255]={0};
    FILE *file;
    FILE *fp=fopen(filename,"w");
    FILE *fp_read=NULL;
    FILE *fp_write=NULL;
    sprintf(name0,"%s_%d",filename,1000);
    sprintf(name1,"%s_%d",filename,1001);
    remove(name0);
    remove(name1);
//    if(!remove(filename))                       //delete file that is open 
//        return false;
//    if(!rename(filename,filename))              //rename file that is open 
//        return false;
    if(!remove(name0))                          //delete file that is not exist
        return false;
    if(!rename(name0,name1))                    //rename file that is not exist
        return false;
    if(NULL!=fopen(name0,"r"))                  //open file that is not exit
        return false;
    file=fopen(name0,"w");  //creat file
    fclose(file);
    fclose(fp);
//    if(!rename(filename,name0))                 //rename file that new name is exist 
//        return false;

    fp=fopen(filename,"r");                      //open file that is open
    if(NULL==fp)
        return false;
    fp_read=fopen(filename,"r");
    if(NULL==fp_read)
        return false;
    if(fclose(fp))
        return false;
    if(fclose(fp_read))
        return false;

    fp=fopen(filename,"w");                     //open file that is open
    if(NULL==fp)
        return false;
    fp_write=fopen(filename,"w");
    if(NULL==fp_write)
        return false;
    if(fclose(fp))
        return false;
    if(fclose(fp_read))
        return false;
   
    if(remove(filename))
        return false;
    if(remove(name0))
        return false;
    
    return true;

}






TEST_F(posix_filesystem, generic){
    FILE* f = NULL;
    char buf[8];
    int ret;

    f = fopen("test1.txt","wb");
    ASSERT_TRUE(f);
    ret = fwrite("hello",1,5,f);
    fclose(f);
    EXPECT_EQ(5,ret);

    f = fopen("test1.txt","rb");
    ASSERT_TRUE(f);
    ret = fread(buf,1,8,f);
    fclose(f);
    EXPECT_EQ(5,ret);
    EXPECT_TRUE(0==memcmp(buf,"hello",5));

    f = fopen("test1.txt","wb");
    ASSERT_TRUE(f);
    ret = fwrite("world",1,5,f);
    fclose(f);
    EXPECT_EQ(5,ret);

    f = fopen("test1.txt","rb");
    ASSERT_TRUE(f);
    ret = fread(buf,1,8,f);
    fclose(f);
    EXPECT_EQ(5,ret);
    EXPECT_TRUE(0==memcmp(buf,"world",5));

}



TEST_F(posix_filesystem, test1){
        //跑完大概需要2-3分钟
    EXPECT_TRUE(test_read_write_file("test_file",1,0,5));
    EXPECT_TRUE(test_read_write_file("test_file",0,0,5));
    EXPECT_TRUE(test_read_write_file("test_file",1,1,5));
    EXPECT_TRUE(test_read_write_file("test_file",0,1,5));
    EXPECT_TRUE(test_tell_seek_file("test_file",5));
    /*
    littlefs api 与 windows api 不一致的地方:
    littlefs
    可以删除已经打开的文件
    可以重命名已经打开的文件
    可以将文件名修改成已经存在的名字
    */
    EXPECT_TRUE(test_remove_rname_file("test_file"));

}


