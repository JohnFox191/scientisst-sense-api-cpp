#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")  // Link with Winsock library
#include <io.h>
#include <stdint.h>
#include "tcp.h"

int initTcpServer(char* port_str){
    int port;
    int listen_fd;
    struct sockaddr_in local_addr;
    int client_fd;
    struct sockaddr_in client_addr;
    socklen_t size_addr = 0;

    sscanf(port_str, "%d", &port);  //Transform port string to int
	
	if((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
		perror("socket: ");
		exit(-1);
	}

	local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);                     
    local_addr.sin_port = htons(port); 

	if(bind(listen_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0){
		perror("bind: ");
		exit(-1);
    }
    printf("Binded port %d on all interfaces\n", port);

	if(listen(listen_fd, 2) == -1){
		perror("listen: ");
		exit(-1);
    }

    //Accept the new client
    if((client_fd = accept(listen_fd, (struct sockaddr*)&client_addr, &size_addr)) == -1){
        perror("accept: ");
        exit(-1);
	}

	return client_fd;
}