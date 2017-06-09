#ifndef __GCS_THREAD_H__
#define __GCS_THREAD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "serialib.h"

int gcs_interface_init(void);
serial * get_local_port(void);


#ifdef __cplusplus
}
#endif

#endif