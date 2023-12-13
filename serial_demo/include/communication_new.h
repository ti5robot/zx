#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#define MAXPENDING 5
#define IDNUM 6
//#include <move/encode.h>
#include<iostream>
using namespace std;
#define PORT 8087
/*struct sockaddr_in listenAddr{};*///server address structure
/*struct sockaddr_in clientAddr{}*/;//client address structure
//uint8_t canidList[IDNUM],cmdList1[IDNUM],cmdList2[IDNUM],cmdList3[IDNUM],cmdList4[IDNUM];
int clientFd;
//void initsock()
//{
//    listenAddr.sin_family = AF_INET;//IPV4 communication domain
//    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);//accept any address
//    listenAddr.sin_port = htons(PORT);//change port to netchar
//};
void die(const char* message) {
    perror(message);
    exit(1);
}
void initsock(const char* NR) 
{

    //if (argc != 2) {
    //    fprintf(stderr, "Usage: %s <Server Port>\n", argv[0]);
    //    exit(1);
    //}

    in_port_t servPort = atoi(NR);  // ��һ�����������ض˿�

    // Ϊ�������Ӵ����׽���
    int servSock;  // ���������׽���������
    if ((servSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        die("socket() failed");

    // ���챾�ص�ַ�ṹ
    struct sockaddr_in servAddr;                  // ���ص�ַ
    memset(&servAddr, 0, sizeof(servAddr));       // ����ṹ
    servAddr.sin_family = AF_INET;                // IPv4��ַϵ��
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY); // �κδ���ӿ�
    servAddr.sin_port = htons(servPort);          // ���ض˿�

    // �󶨵����ص�ַ
    if (bind(servSock, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0)
        die("bind() failed");

    // ���socket��ʹ���ܹ�������������
    if (listen(servSock, MAXPENDING) < 0)
        die("listen() failed");

    printf("Waiting for connections...\n");

    for (;;) {
        // ���ܿͻ�������
        struct sockaddr_in clntAddr; // �ͻ��˵�ַ
        socklen_t clntAddrLen = sizeof(clntAddr);

        int clntSock = accept(servSock, (struct sockaddr*)&clntAddr, &clntAddrLen);
        if (clntSock < 0)
            die("accept() failed");

        // clntSock�����ӵ��ͻ��ˣ�

        char clntName[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &clntAddr.sin_addr.s_addr, clntName,
            sizeof(clntName)) != NULL)
            printf("Handling client %s/%d\n", clntName, ntohs(clntAddr.sin_port));
        else
            puts("Unable to get client address");

        char   buffer[BUFSIZ];

        // �������Կͻ��˵���Ϣ
        ssize_t numBytesRcvd = recv(clntSock, buffer, BUFSIZ, 0);
        cout <<buffer<< endl;
        if (numBytesRcvd < 0)
            die("recv() failed");
        
        // ���ͽ��յ����ַ������ٴν��գ�ֱ���������
        while (numBytesRcvd > 0)
        { // ���ʾ�������
            // ������Ϣ���ͻ���
            if (send(clntSock, buffer, numBytesRcvd, 0) != numBytesRcvd)
                die("send() failed");

            // �������Կͻ��˵���Ϣ
            numBytesRcvd = recv(clntSock, buffer, BUFSIZ, 0);
            if (numBytesRcvd < 0)
                die("recv() failed");
        }

        close(clntSock); // �رտͻ����׽���
    }

    // NOT REACHED


}
