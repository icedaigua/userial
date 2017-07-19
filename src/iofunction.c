#include "iofunction.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>

// 

int main(int argc,char *argv[]){

    double posi[3] = {0.0};
    if(getPositionFromfile(3,posi) == -1){
        return -1;
    };

    printf("posi = %16.7f %16.7f %10.3f\n",posi[0],posi[1],posi[2]);
    
    if(savePositionTofile(7,posi) == -1){
        return -1;
    };

    return 0;
}

int savePositionTofile(unsigned short index,double *posi){
    FILE *fp;
    int i = 0;
    char buf[120];			// 每行不超过120字节, 根据情况调节大小
        
    if (!(fp = fopen("./flightpoints.txt", "r+"))) {		// 尝试以读写方式打开文件.
        fprintf(stderr, "Open failed.\n");
        return -1;
     }
     for (i = 0; i < index-1; i++) {              // 循环index-1次, 读掉前index-1行
           fgets(buf, 120, fp);	                // 读取一行
    }
    // 此时文件指针指向第index行行首
    long offset = ftell(fp);	                // 记录文件指针位置, 因为后面还要读, 文件指针会移走
    
    
    //设后面不超过20行, 每行不超过120字节
    char save[20][120];						 
        
    i = 0;                                         // 清0, 记录后面共有多少行
    while ((fgets(save[i], 1024, fp))) {	       // 循环读取文件, 直到fgets返回NULL表示读完
           i++;
    }

    sprintf(buf,"%3d %16.8f %16.8f %10.4f\n",index,posi[0],posi[1],posi[2]);

    // 由于读完文件后, 文件指针指向文件尾, 这里重新定位到之前保存的位置
    fseek(fp, offset, SEEK_SET);       
    fputs(buf, fp);									// 写要插入的数据
    int j = 0;
    for (j = 1; j < i; j++) {					// 之前保存的数据, 依次往后面写
        fputs(save[j], fp);
    }

    fclose(fp);
    return 0;
}



int getPositionFromfile(unsigned short index,double *posi){
  char posiBuff[120],temp[120];
  int kc=index-1;//j=4,k=0;    //第三行，第四列
  FILE *fp = fopen("./flightpoints.txt","r");
  if(!fp)
  {
        fprintf(stderr, "Open failed.\n");
        return -1;
  }
  while(fgets(temp,120,fp)){    //读入每行数据
    if(kc==0){
        strcpy(posiBuff,temp);    //读到第三行数据
        break;
    }
    kc--;
    //b[k++]=c[j-1];        //把每行的那列字符拷到b中
  }

  sscanf(posiBuff,"%3d %lf %lf %lf",&kc,&posi[0],&posi[1],&posi[2]);
  //b[k]=0;
//   printf("第%d行数据：%s\n",i,a);
//   printf("第%d列数据：%s\n",j,b);
  fclose(fp);

  return 0;
}



//int getPositionFromfile(unsigned short index,double *posi) {
//         char s[] = "main.c";
//         //只读打开文件
//         int fd = open(s, O_RDONLY);

//         if(fd == -1) {
//                 printf("error is %s\n", strerror(errno));
//                 return -1;
//         }

//         printf("sucess fd = %d\n", fd);
//         char buf[100];
//         memset(buf, 0, sizeof(buf));
//         //read返回0表示文件读取完毕
//         while(read(fd, buf, sizeof(buf) - 1) > 0) {
//                 printf("%s\n", buf);
//                 memset(buf, 0, sizeof(buf));
//         }

//         printf("%s\n", buf);
//         //别忘记关闭
//         close(fd);

//         return 0;
// }

// int savePositionTofile(unsigned short index,double *posi) {
//         char s[] = "abc.txt";
//         //读写并追加方式
//         int fd = open(s, O_RDWR | O_APPEND);

//         if(fd == -1) {
//                 //发生错误会把错误信息设置到errno中
//                 //通过strerror函数获取错误内容
//                 printf("error is %s\n", strerror(errno));
//                 return -1;
//         }

//         printf("sucess fd = %d\n", fd);
//         char buf[100];
//         memset(buf, 0, sizeof(buf));
//         int i = 10;
//         while(i-- > 0){
//                 strcpy(buf, "hello world linux write file \n");
//                 write(fd, buf, strlen(buf));
//                 memset(buf, 0, sizeof(buf));
//         }

//         close(fd);
//         return 0;
// }
