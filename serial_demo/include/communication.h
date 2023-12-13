#include <arpa/inet.h>
#include <sys/socket.h>
#define IDNUM 6
//#include <move/encode.h>
#include<iostream>
using namespace std;
#define PORT 8087
struct sockaddr_in listenAddr{};//server address structure
struct sockaddr_in clientAddr{};//client address structure
//uint8_t canidList[IDNUM],cmdList1[IDNUM],cmdList2[IDNUM],cmdList3[IDNUM],cmdList4[IDNUM];
int clientFd;
void initsock()
{
    listenAddr.sin_family = AF_INET;//IPV4 communication domain
    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);//accept any address
    listenAddr.sin_port = htons(PORT);//change port to netchar
};
