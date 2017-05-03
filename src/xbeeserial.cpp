
#include "xbeeserial.h"

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>  /*数据类型，比如一些XXX_t的那种*/
#include <sys/stat.h>   /*定义了一些返回值的结构，没看明白*/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/



int serialOpen(void){
    int fd;

    /*以读写方式打开串口*/

    fd = open( "/dev/ttyUSB0", O_RDWR);

    if (-1 == fd){

    /* 不能打开串口一*/

    perror(" 提示错误！");

    }

    return fd;
}


/**

*@brief  设置串口通信速率

*@param  fd     类型 int  打开串口的文件句柄

*@param  speed  类型 int  串口速度

*@return  void

*/

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,

                  B38400, B19200, B9600, B4800, B2400, B1200, B300, };

int name_arr[] = {115200,38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,

                  19200,  9600, 4800, 2400, 1200,  300, };

void serialParameters(int fd, int baud){

    int   i;
    int   status;

    struct termios   Opt;

    tcgetattr(fd, &Opt);

    struct termios options;  // 串口配置结构体
    tcgetattr(fd,&options); //获取当前设置
//    bzero(&options,sizeof(options));
    options.c_cflag  |= B115200 | CLOCAL | CREAD ; // 设置波特率，本地连接，接收使能
    options.c_cflag &= ~CSIZE; //屏蔽数据位
    options.c_cflag  |= CS8; // 数据位为 8 ，CS7 for 7
    options.c_cflag &= ~CSTOPB; // 一位停止位， 两位停止为 |= CSTOPB
    options.c_cflag &= ~PARENB; // 无校验
     //options.c_cflag |= PARENB; //有校验
    //options.c_cflag &= ~PARODD // 偶校验
    //options.c_cflag |=  PARODD    // 奇校验
    options.c_cc[VTIME] = 0; // 等待时间，单位百毫秒 （读）。后有详细说明
    options.c_cc[VMIN] = 0; // 最小字节数 （读）。后有详细说明
    tcflush(fd, TCIOFLUSH); // TCIFLUSH刷清输入队列。
//                                           TCOFLUSH刷清输出队列。
//                                           TCIOFLUSH刷清输入、输出队列。
    tcsetattr(fd, TCSANOW, &options); // TCSANOW立即生效；
//                                                           TCSADRAIN：Wait until everything has been transmitted；
//                                                            TCSAFLUSH：Flush input and output buffers and make the change

}
