#ifndef _UDP_H
#define _UDP_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h> 
#include <unistd.h>
#include <netdb.h>

int initUdpServer(char* port_str, struct sockaddr_in *client_addr, socklen_t *client_addr_len);

#endif