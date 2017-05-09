
#include "gcs_thread.h"

#include <stdio.h>


int main(int argc,char** argv)
{
    if(init_serial()==-1)
        return 0;
    init_sigaction();
	init_time();

    while(1){}


}