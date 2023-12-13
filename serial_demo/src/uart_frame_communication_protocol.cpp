#include "uart_frame_communication_protocol.h"
#include "actuator_control_interface.h"
#include "mainwindow.h"
int32_t crc32_table[256]={0,1996959894,-301047508,-1727442502,124634137,1886057615,-379345611,-1637575261,	249268274,2044508324,-522852066,-1747789432,162941995,2125561021,-407360249,-1866523247,498536548,1789927666,-205950648,-2067906082,450548861,1843258603,-187386543,-2083289657,325883990,1684777152,-43845254,-1973040660,	335633487,1661365465,-99664541,-1928851979,997073096,1281953886,-715111964,-1570279054,1006888145,1258607687,-770865667,-1526024853,901097722,1119000684,-608450090,-1396901568,853044451,1172266101,-589951537,-1412350631,	651767980,1373503546,-925412992,-1076862698,565507253,1454621731,-809855591,-1195530993,671266974,1594198024,-972236366,-1324619484,795835527,1483230225,-1050600021,-1234817731,1994146192,31158534,-1731059524,-271249366,	1907459465,112637215,-1614814043,-390540237,2013776290,251722036,-1777751922,-519137256,2137656763,141376813,-1855689577,-429695999,1802195444,476864866,-2056965928,-228458418,1812370925,453092731,-2113342271,-183516073,	1706088902,314042704,-1950435094,-54949764,1658658271,366619977,-1932296973,-69972891,1303535960,984961486,-1547960204,-725929758,1256170817,1037604311,-1529756563,-740887301,1131014506,879679996,-1385723834,-631195440,	1141124467,855842277,-1442165665,-586318647,1342533948,654459306,-1106571248,-921952122,1466479909,544179635,-1184443383,-832445281,1591671054,702138776,-1328506846,-942167884,1504918807,783551873,-1212326853,-1061524307,-306674912,-1698712650,62317068,1957810842,-355121351,-1647151185,81470997,1943803523,-480048366,-1805370492,225274430,2053790376,-468791541,-1828061283,167816743,2097651377,-267414716,-2029476910,503444072,1762050814,-144550051,-2140837941,426522225,1852507879,-19653770,-1982649376,282753626,1742555852,-105259153,-1900089351,397917763,1622183637,-690576408,-1580100738,953729732,1340076626,-776247311,-1497606297,1068828381,1219638859,-670225446,-1358292148,906185462,1090812512,-547295293,-1469587627,829329135,1181335161,-882789492,-1134132454,628085408,1382605366,-871598187,-1156888829,570562233,1426400815,-977650754,-1296233688,733239954,1555261956,-1026031705,-1244606671,752459403,1541320221,-1687895376,-328994266,1969922972,40735498,-1677130071,-351390145,1913087877,83908371,-1782625662,-491226604,2075208622,213261112,-1831694693,-438977011,2094854071,198958881,-2032938284,-237706686,1759359992,534414190,-2118248755,-155638181,1873836001,414664567,-2012718362,-15766928,1711684554,285281116,-1889165569,-127750551,1634467795,376229701,-1609899400,-686959890,1308918612,956543938,-1486412191,-799009033,1231636301,1047427035,-1362007478,-640263460,1088359270,936918000,-1447252397,-558129467,1202900863,817233897,-1111625188,-893730166,1404277552,615818150,-1160759803,-841546093,1423857449,601450431,-1285129682,-1000256840,1567103746,711928724,-1274298825,-1022587231,1510334235,755167117};

UFCP::UFCP()
{

}

uint32_t calcCRC32(uint32_t crc, uint8_t *string, uint32_t size)
{
    while(size--)
        crc = (crc >> 8)^(crc32_table[(crc ^ *string++)&0xff]);
    return crc;
}




int16_t table_s[]={12336,12592,12848,13104,13360,13616,13872,14128,14384,14640,16688,16944,17200,17456,17712,17968,12337,12593,12849,13105,13361,13617,13873,14129,14385,14641,16689,16945,17201,17457,17713,17969,12338,12594,12850,13106,13362,13618,13874,14130,14386,14642,16690,16946,17202,17458,17714,17970,12339,12595,12851,13107,13363,13619,13875,14131,14387,14643,16691,16947,17203,17459,17715,17971,12340,12596,12852,13108,13364,13620,13876,14132,14388,14644,16692,16948,17204,17460,17716,17972,12341,12597,12853,13109,13365,13621,13877,14133,14389,14645,16693,16949,17205,17461,17717,17973,12342,12598,12854,13110,13366,13622,13878,14134,14390,14646,16694,16950,17206,17462,17718,17974,12343,12599,12855,13111,13367,13623,13879,14135,14391,14647,16695,16951,17207,17463,17719,17975,12344,12600,12856,13112,13368,13624,13880,14136,14392,14648,16696,16952,17208,17464,17720,17976,12345,12601,12857,13113,13369,13625,13881,14137,14393,14649,16697,16953,17209,17465,17721,17977,12353,12609,12865,13121,13377,13633,13889,14145,14401,14657,16705,16961,17217,17473,17729,17985,12354,12610,12866,13122,13378,13634,13890,14146,14402,14658,16706,16962,17218,17474,17730,17986,12355,12611,12867,13123,13379,13635,13891,14147,14403,14659,16707,16963,17219,17475,17731,17987,12356,12612,12868,13124,13380,13636,13892,14148,14404,14660,16708,16964,17220,17476,17732,17988,12357,12613,12869,13125,13381,13637,13893,14149,14405,14661,16709,16965,17221,17477,17733,17989,12358,12614,12870,13126,13382,13638,13894,14150,14406,14662,16710,16966,17222,17478,17734,17990};
uint32_t MemToStrStream(char* dest, const char* sourse, uint32_t size,char head,char end)
{
    uint32_t i = 0;
    uint8_t *pindex = (uint8_t *)sourse;
    *dest = head;
    int16_t *p = (int16_t *)(dest+1);
    for(i=0;i<size;i++)
    {
        p[i] = table_s[pindex[i]];
    }
    p[i] = end;
    //    qDebug()<<"end:"<<dest[size*2+1]<<dest[size*2+2];
    //    printf("**%d**%d**,",dest[size*2+1],dest[size*2+2]);
    return size*2+2;
}
int32_t sendCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
    uint8_t buf[400],index=0;
    buf[index++] = numOfActuator;
//    qDebug()<<"buf[0]:"<<buf[0];
    if(numOfActuator==0)
    {
        buf[index++] = canIdList[0];
        buf[index++] = commandList[0];
        buf[index] = parameterList[0];
        index +=4;
    }
    else
    {
        for(int i=0;i<numOfActuator;i++)
        {
            buf[index++] = canIdList[i];
        }
        for(int i=0;i<numOfActuator;i++)
        {
            buf[index++] = commandList[i];
//            qDebug()<<commandList[i];
        }
        for(int i=0;i<numOfActuator;i++)
        {

            *(uint32_t *)(buf+index) = parameterList[i];
            index+=4;
        }
    }

    *(uint32_t *)(buf+index) = calcCRC32(0,buf,index);
    //    qDebug()<<*(uint32_t *)(buf+index);
    index+=4;
    char uartTxBuf[2000];
    uint32_t byteToWrite = MemToStrStream(uartTxBuf,(char*)buf,index,'<','\n');
    ulong writeSize=0;
    if(Serial::write(uartTxBuf, byteToWrite,&writeSize))
    {
        qDebug()<<"write serial port error.";
        return -1;
    }
    else
    {
//        qDebug()<<"send:"<<uartTxBuf;
        if(byteToWrite != writeSize)
        {
            //qDebug()<<"write bytes less.";
            return -1;
        }
    }
    return 0;


}

void UFCP::qt_write_csv_test()
{
    //qDebug()<<"--------";
   QFile outFile("D:/1.csv");
   QStringList lines;
   lines <<QString::number(ActuatorList[0].Position)<<","<<QString::number(ActuatorList[1].Position)<<","<<QString::number(ActuatorList[2].Position)<<"\n";
   //lines << "00,01,02\n" << "10,11,12\n" << "20,21,22";
   /*如果以ReadWrite方式打开，不会把所有数据清除，如果文件原来的长度为100字节，写操作写入了30字节，那么还剩余70字节，并且这70字节和文件打开之前保持一直*/
   if (outFile.open(QIODevice::Append))
   {
    for (int i = 0; i < lines.size(); i++)
    {
        outFile.write(lines[i].toStdString().c_str());/*写入每一行数据到文件*/
    }
    outFile.close();
   }
}




int32_t UFCP::readCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
    char uartRxbuf[2000]={0};
    ulong bytesToRead=0;
    int res = 0;
    if(this->read(uartRxbuf, 5000, &bytesToRead))
    {
//        qDebug()<<"recv result:"<<false;
//        return res;
    }
//    if(bytesToRead>0)
//        qDebug()<<uartRxbuf;
    static QString strStream;
    char uartRxBuf[512] = {0};
    memset(uartRxBuf,0,500);
    this->stringBuffer.append(QString(uartRxbuf));





    while(true)
    {
        int index = this->stringBuffer.indexOf("\n");
        if(index<0)
            break;
        else if(index == 0)
            this->stringBuffer.remove(0,1);
        else
        {
            uint8_t mem[400]={0},memIndex=0;
            QString frame = this->stringBuffer.left(index);//去掉'\n'
//            qDebug()<<frame;
            this->stringBuffer.remove(0,index+1);
            if(frame.at(0) == '<')
            {
                frame.remove(0,1);
                int size = index-1;
                while(true)
                {
                    if(frame.size()>1)
                    {
                        bool res;
                        mem[memIndex] = frame.left(2).toInt(&res,16);
                        memIndex++;
                        if(!res)
                        {
                            strStream.clear();
                            qDebug()<<"HextoInt erro"<<frame.left(2)<<" char "<<mem[memIndex-1];
                            break;
                        }
                        else
                        {
                            frame.remove(0,2);
                        }
                    }
                    else
                    {
                        uint32_t crcRes = this->calcCRC32(0,mem,memIndex-4);
//                        qDebug()<<"crc:"<<*(uint32_t*)(mem+memIndex-4)<<"calc crc:"<<crcRes;
                        if(*(uint32_t*)(mem+memIndex-4) == crcRes)
                        {
                            numOfActuator = mem[0];
                            res = numOfActuator;
//                            qDebug()<<"num of find:"<<numOfActuator;
//                            qDebug()<<(char*)mem;
                            uint8_t *pCanId = mem+1;
                            uint8_t *pCommand = mem+numOfActuator+1;
                            uint8_t *pParameter = mem+numOfActuator+numOfActuator+1;
                            for(int i=0;i<numOfActuator;i++)
                            {
                                canIdList[i]=mem[i+1];
                                commandList[i]=mem[i+numOfActuator+1];
                                parameterList[i]=mem[i*4+numOfActuator*2+1];


                                switch (*pCommand++)
                                {
                                case GET_ACT_INFO:
                                {
                                    qDebug()<<"id:"<<canIdList[i]<<"cmd:"<<commandList[i]
                                              <<"parameter:"<<parameterList[i];
                                    IdToIndexMap[*pCanId] = i;
                                    ActuatorList[i].CanId = *pCanId++;
                                    ActuatorList[i].OnlineState = 1;
                                    ActuatorList[i].Infomation = *(uint32_t*)pParameter;

                                }
                                    break;
                                case GET_REG_CSP:
                                {
                                    uint8_t index = IdToIndexMap[*pCanId++];
                                    ActuatorList[index].Iq_mA = *(int16_t*)pParameter;
                                    pParameter+=2;
                                    ActuatorList[index].Speed = *(int16_t*)pParameter;
                                    pParameter+=2;
                                    ActuatorList[index].Position = *(int32_t*)pParameter;
                                    pParameter+=4;


                                    if (abs(ActuatorList[index].Speed)>1){
                                         if(ActuatorList[0].Mode==1){
//                                        qt_write_csv_test();
                                         }


                         int32_t      x =      (int32_t)4294967237;
                         if(ActuatorList[index].CanId==2){


                             qDebug()<<ActuatorList[index].Iq_mA;
                                 }
                                    }
                                    }
                                    break;
                                default:
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }

    }


    return res;
}
/*
bool UFCP::openSerialPort(char *portname, uint32_t baudRate, uint8_t byteSize)
{
    char str[10]={0};
    WCHAR wszClassName[10]={0};
    sprintf(str,"\\\\.\\%s",portname);
    MultiByteToWideChar(CP_ACP, 0, str, strlen(str) , wszClassName, sizeof(wszClassName) / sizeof(wszClassName[0]));
    this->hCom = CreateFile(wszClassName,//COM1口
        GENERIC_READ | GENERIC_WRITE, //允许读和写
        0, //独占方式
        NULL,
        OPEN_EXISTING, //打开而不是创建
        0, //同步方式
        NULL);
    if (this->hCom == INVALID_HANDLE_VALUE)
    {
//        qDebug()<<"open serial failed!\n";
        return false;
    }
    else
    {
//        qDebug()<<"open serial succeed!\n";
    }
    SetupComm(this->hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
    COMMTIMEOUTS TimeOuts;
    //设定读超时
    TimeOuts.ReadIntervalTimeout = 1;
    TimeOuts.ReadTotalTimeoutMultiplier = 0;
    TimeOuts.ReadTotalTimeoutConstant = 2;
    //设定写超时
    TimeOuts.WriteTotalTimeoutMultiplier = 0;
    TimeOuts.WriteTotalTimeoutConstant = 2;
    SetCommTimeouts(this->hCom, &TimeOuts); //设置超时
    DCB dcb;
    GetCommState(this->hCom, &dcb);
    dcb.BaudRate = baudRate; //波特率
    dcb.ByteSize = byteSize; //每个字节有8位
    dcb.Parity = NOPARITY; //无奇偶校验位
    dcb.StopBits = ONESTOPBIT; //1个停止位
    SetCommState(this->hCom, &dcb);
    return true;
}

bool UFCP::read(char *readBuf, const uint32_t byteToRead, unsigned long *readSize)
{

}

bool UFCP::write(char *writeBuf, const uint32_t byteToWrite, unsigned long *writeSize)
{

}
*/

//void CanFrame_t::clear()
//{
//    memset(this,0,sizeof(CanFrame_t));
//}
