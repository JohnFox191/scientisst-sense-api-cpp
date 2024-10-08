#ifndef _UDP_H
#define _UDP_H

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")  // Link with Winsock library
#include <io.h>
#include <stdint.h>

int initUdpServer(char* port_str, struct sockaddr_in *client_addr, socklen_t *client_addr_len);

#endif