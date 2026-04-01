#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include "st_sc_servo_control_lib/SCS.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);

protected:
	int writeSCS(unsigned char *nDat, int nLen);
	int readSCS(unsigned char *nDat, int nLen);
	int writeSCS(unsigned char bDat);
	void rFlushSCS();
	void wFlushSCS();
public:
	unsigned long int IOTimeOut;
	int Err;
public:
	virtual int getErr(){  return Err;  }
	virtual int setBaudRate(int baudRate);
	virtual bool begin(int baudRate, const char* serialPort);
	virtual void end();
protected:
    int fd;
    struct termios orgopt;
	struct termios curopt;
	unsigned char txBuf[255];
	int txBufLen;
};

#endif