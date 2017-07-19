#include "protocol.h"
#include "gcs_thread.h"


#include <pthread.h>
#include <stdio.h>

#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <signal.h>
#include <sys/time.h>


void init_time(void) ;
void init_sigaction(void);
int init_serial(void);
void thread_run(void);
void setUAVTest(void);

void *thread2();
void *thread1();

uint32_t number = 0;
pthread_t thread[2];
pthread_mutex_t mut;

uint32_t limit = 0;

int rec_len = 0;

serial *s;

uint8_t buffer[128];

void timeout_info(int signo) {

    int kc =0;
	rec_len = serial_read(s, buffer, '\n', 128);

    //if(rec_len>5)
    {
        printf("rec len = %d\n",rec_len);
        // for(kc = 0;kc<rec_len;kc++)
        // {
        //     printf("%x    ",buffer[kc]);
        // }

        // printf("\n");

        received_task(buffer,rec_len);
    }
        
    CommProtocol_task();
    //printf("time is %10.3f \n",(limit++)*0.1);
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

	val.it_value.tv_sec = 0;
	val.it_value.tv_usec = 100000;
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
    return 0;
}


void *thread1()
{
    printf ("thread1 : I'm thread 1\n");
    while(1)
    {
    //    printf("thread1 : number = %d\n",number);
    //    pthread_mutex_lock(&mut);
    //    number++;
    //    pthread_mutex_unlock(&mut);
        setUAVTest();
        usleep(100*1000);
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
   memset(&thread, 0, sizeof(thread));          //comment1
        /*创建线程*/
   if(( pthread_create(&thread[0], NULL, thread1, NULL)) != 0)       //comment2
       printf("线程1 创建失败!\n");
   else
       printf("线程1 被创建\n");

//    if((pthread_create(&thread[1], NULL, thread2, NULL)) != 0)  //comment3
//         printf("线程2 创建失败");
//    else
//         printf("线程2 被创建\n");
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


int gcs_interface_init(void){
    
    if(init_serial()==-1)
        return -1;
    init_sigaction();
	init_time();

    CommProtocol_init();

    return 0;
}


serial * get_local_port(void){
    return s;
}


void setUAVTest(void)
{
    static uint32_t time = 0;
    static int number = -1;
    static uint8_t cnt = 0;
    BroadcastDataU boardData = {0};

    boardData.timeStamp.time = time++;

   
    if(cnt++ >=100){
        printf("number = %d \n",number);
        setImageStatus(&number);
        
        if((number++)>=9) number = 0;
        cnt = 0;
    }
    boardData.ctrlInfo.mode = 11;
    boardData.status = 3;
    
    setUAVstatus((uint8_t*)&boardData,sizeof(BroadcastDataU));
}