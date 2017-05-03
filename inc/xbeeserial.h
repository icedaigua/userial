#ifndef __XBEE_SERIAL_H__
#define __XBEE_SERIAL_H__

int serialOpen(void);

void serialParameters(int fd, int baud);

#endif
