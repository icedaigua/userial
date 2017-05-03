#include "xbeeserial.h"


#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>  /*数据类型，比如一些XXX_t的那种*/
#include     <sys/stat.h>   /*定义了一些返回值的结构，没看明白*/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/


char sendbuf[50]="ABCDE123456";

int main(int argc,char **argv){

    int fd = 0;

    fd = serialOpen();

//    printf("serial is %d \n",fd);

//    serialParameters(fd,115200);

    while(true){
        printf("send buf\n");
        write(fd,sendbuf,5);
        sleep(2);
    }


}
