#include "gcs_thread.h"
#include "serialib.h"

#include <pthread.h>
#include <stdio.h>

#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <signal.h>
#include <sys/time.h>


uint32_t number = 0;
pthread_t thread[2];
pthread_mutex_t mut;

uint32_t limit = 0;

serial *s;

char buffer[128];

void timeout_info(int signo) {

	serial_read(s, buffer, '\n', 128);
	printf("%s %d\n", buffer, strlen(buffer));
		
	serial_write(s, "ls\r\n");
}

/* init sigaction */
void init_sigaction(void) {
	struct sigaction act;

	act.sa_handler = timeout_info;
	act.sa_flags = 0;
	sigemptyset(&act.sa_mask);
	sigaction(SIGPROF, &act, NULL);
}

/* init */
void init_time(void) {
	struct itimerval val;

	val.it_value.tv_sec = 2;
	val.it_value.tv_usec = 0;//100000;
	val.it_interval = val.it_value;
	setitimer(ITIMER_PROF, &val, NULL);
}

int init_serial(void){

	if (serial_open(&s, "/dev/ttyUSB0", 115200) == 0){
		printf("Port opened.\n");

	} else {
		printf("Problem with port opening\n");
		return -1;
	}
	printf("%s -> %d\n", s->port, s->fd);

}


void *thread1()
{
    printf ("thread1 : I'm thread 1\n");
    while(1)
    {
       printf("thread1 : number = %d\n",number);
       pthread_mutex_lock(&mut);
       number++;
       pthread_mutex_unlock(&mut);
       sleep(2);
    }
   printf("thread1 :主函数在等我完成任务吗？\n");
   pthread_exit(NULL);
}

void *thread2()
{
    printf("thread2 : I'm thread 2\n");
    while(1)
    {
        printf("thread2 : number = %d\n",number);
        pthread_mutex_lock(&mut);
        number++;
        pthread_mutex_unlock(&mut);
        sleep(3);
    }
    // for (i = 0; i < MAX; i++)
    // {
    //    printf("thread2 : number = %d\n",number);
    //    pthread_mutex_lock(&mut);
    //    number++;
    //    pthread_mutex_unlock(&mut);
    //    sleep(3);
    // }
    printf("thread2 :主函数在等我完成任务吗？\n");
   pthread_exit(NULL);
}

void thread_create(void)
{
    // int temp;
   memset(&thread, 0, sizeof(thread));          //comment1
        /*创建线程*/
   if(( pthread_create(&thread[0], NULL, thread1, NULL)) != 0)       //comment2
       printf("线程1 创建失败!\n");
   else
       printf("线程1 被创建\n");

   if((pthread_create(&thread[1], NULL, thread2, NULL)) != 0)  //comment3
        printf("线程2 创建失败");
   else
        printf("线程2 被创建\n");
}

void thread_wait(void)
{
        /*等待线程结束*/
   if(thread[0] !=0) 
    {                  
       pthread_join(thread[0],NULL);
       printf("线程1 已经结束\n");
    }
   if(thread[1] !=0) 
    {   
        pthread_join(thread[1],NULL);
        printf("线程2 已经结束\n");
    }
}

void thread_run(void){
   /*用默认属性初始化互斥锁*/
    pthread_mutex_init(&mut,NULL);
    printf("我是主函数，我正在创建线程，呵呵\n");
    thread_create();
    printf("我是主函数，我正在等待线程完成任务阿，呵呵\n");
    thread_wait();
}
