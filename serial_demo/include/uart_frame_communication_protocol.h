#ifndef UART_FRAME_COMMUNICATION_PROTOCOL_H
#define UART_FRAME_COMMUNICATION_PROTOCOL_H
#include <stdint.h>
#include <QString>
#include <QDebug>
//#include "cantransfer.h"
#include "serial.h"
class UFCP : public Serial
{
public:
    UFCP();
    Serial serialPort;
    uint32_t calcCRC32(uint32_t crc, uint8_t *string, uint32_t size);
    QString strStream;


//    bool openSerialPort(char *portname, uint32_t baudRate=2000000, uint8_t byteSize=8);
//    bool read(char *readBuf, const uint32_t byteToRead, unsigned long *readSize);
//    bool write(char *writeBuf, const uint32_t byteToWrite, unsigned long *writeSize);
    int32_t sendCommand(uint8_t numOfActuator,uint8_t *canIdList,uint8_t *commandList,uint32_t *parameterList);
    int32_t readCommand(uint8_t numOfActuator,uint8_t *canIdList,uint8_t *commandList,uint32_t *parameterList);
    void qt_write_csv_test();

private:
    HANDLE hCom;
    QString stringBuffer;
};

#endif // UART_FRAME_COMMUNICATION_PROTOCOL_H
