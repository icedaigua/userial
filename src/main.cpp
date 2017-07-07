
#include "gcs_thread.h"
#include "protocol.h"

// #include<sys/time.h>
#include <stdio.h>
#include <unistd.h>

void setUAVTest(void);

int main(int argc,char** argv)
{
    if(gcs_interface_init()==-1)
        return 0;

    thread_create();
    while(1){

    }


}


