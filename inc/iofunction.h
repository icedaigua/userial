#ifndef __IO_FUNCTION_H__
#define __IO_FUNCTION_H__


#ifdef __cplusplus
extern "C" {
#endif



int savePositionTofile(unsigned short index,double *posi);


int getPositionFromfile(unsigned short index,double *posi);


#ifdef __cplusplus
}
#endif


#endif