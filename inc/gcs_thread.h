#ifndef __GCS_THREAD_H__
#define __GCS_THREAD_H__


void init_time(void) ;
void init_sigaction(void);
int init_serial(void);
void thread_run(void);

#endif