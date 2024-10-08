#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "udp.h"


int initUdpServer(char* port_str, struct sockaddr_in *client_addr, socklen_t *client_addr_len){
    uint8_t buff[255];
    int port;
    int client_fd;
    struct sockaddr_in local_addr;

    sscanf(port_str, "%d", &port);  //Transform port string to int
	
	if((client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		perror("socket: ");
		exit(-1);
	}


	local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);                     
    local_addr.sin_port = htons(port); 

	if(bind(client_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0){
		perror("bind: ");
		exit(-1);
    }
	printf("Binded port %d on all interfaces\n", port);

    if(recvfrom(client_fd, (char*)buff, 255, 0, (struct sockaddr*)client_addr, client_addr_len) < 0){
		perror("recvfrom: ");
		exit(-1);
    }
	client_addr->sin_family = AF_INET;

	printf("UDP handshake done\n");

	return client_fd;
}