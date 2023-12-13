#define IDNUM 6
#define BUF_SIZE (7*IDNUM+3)
#include<iostream>
using namespace std;
uint32_t calcSum(uint32_t crc, uint8_t *string, uint32_t size)
{
    for(int i=0;i<size;i++) crc += string[i];
    crc = crc & 0xFF;
    return crc;
};
uint32_t ASCToHex(uint8_t *ASC)
{
    uint8_t i=0;
    uint8_t j=0;
    uint8_t Hex[4];
    uint8_t ASCcodeTable1[17]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46,0x20};
    uint8_t inter[21]={0};
    for(j=9;j<21;j++)
        for(i=0;i<17;i++)
            if(ASC[j] == ASCcodeTable1[i])
				inter[j]=i;
    for(i=0;i<4;i++)
        Hex[i]=inter[9+3*i]*16+inter[3*i+10];
    return *(uint32_t *)Hex;  
};
void HexToASC(uint8_t *ASC, uint8_t *Hex,uint8_t bl)
{
    uint8_t i=0;
    uint8_t j=0;
    uint8_t ASCcodeTable1[17]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46,0x20};
    uint8_t inter[3*bl]={0};
    
    for(i=0,j=0;i<bl;i++){
	inter[j++]=Hex[i]/16;
        inter[j++]=Hex[i]%16;
        inter[j++]=16;
    }
    for(j=0;j<(bl*3);j++)
        for(i=0;i<17;i++)
            if(inter[j] == i)
                ASC[j]=ASCcodeTable1[i];
   
};
void package_save_cmd(uint8_t *ASC,uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList)
{
    uint8_t buf[BUF_SIZE]={0};
    uint32_t index=0;
    buf[index++] = 0xFF;
    buf[index++] = numOfActuator;
    for(int i=0;i<numOfActuator;i++){
        buf[index++] = canIdList[i];
	buf[index++] = 0x01;
	buf[index++] = commandList[i];
	index+=4;
    }
    *(uint32_t *)(buf+index) = calcSum(0,buf,index);
    index++;
    HexToASC(ASC,buf,BUF_SIZE);
};
void package_void_cmd(uint8_t *ASC,uint8_t canId, uint8_t command)
{
    uint8_t buf[12]={0};
    uint32_t index=0;
    buf[index++] = 0xFF;
    buf[index++] = 0x01;
    buf[index++] = canId;
    buf[index++] = 0x01;
    buf[index++] = command;
    //*(uint32_t *)(buf+index) = 0;
    index+=4;
    *(uint32_t *)(buf+index) = calcSum(0,buf,index);
    index++;
    HexToASC(ASC,buf,index);
};
void package_set_cmd(uint8_t *ASC,uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
    uint8_t buf[BUF_SIZE]={0};
    uint32_t index=0;
    buf[index++] = 0xFF;
    buf[index++] = numOfActuator;
    for(int i=0;i<numOfActuator;i++){
        buf[index++] = canIdList[i];
        buf[index++] = 0x05;
        buf[index++] = commandList[i];
        *(uint32_t *)(buf+index) = parameterList[i];
        index+=4;
    } 
    *(uint32_t *)(buf+index) = calcSum(0,buf,index);
    index++;
    HexToASC(ASC,buf,BUF_SIZE);
};
