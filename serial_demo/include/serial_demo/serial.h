#ifndef SERIAL_H
#define SERIAL_H
#include <stdint.h>
//#include <iostream>
#include <string>
#include <ros/ros.h>

class Serial
{
public:
    Serial();
    //bool open(char *portname, uint32_t baudRate=2000000, uint8_t byteSize=8);
    void open(char *portname, uint32_t baudRate=2000000, uint8_t byteSize=8);
    void read(char *readBuf, const uint32_t byteToRead, unsigned long *readSize);
    void write(char *writeBuf, const uint32_t byteToWrite, unsigned long *writeSize);
//    inline bool write(uint8_t *writeBuf, const uint32_t byteToWrite, unsigned long *writeSize);

    //static hCom;


};

#endif // SERIAL_H

