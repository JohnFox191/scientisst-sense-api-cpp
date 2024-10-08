#ifdef _WIN32 // 32-bit or 64-bit Windows

//#define HASBLUETOOTH

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <winsock2.h>
#include <ws2bth.h>

#else // Linux or Mac OS

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/* #ifdef HASBLUETOOTH  // Linux only

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <stdlib.h>

#endif // HASBLUETOOTH
 */
void Sleep(int millisecs)
{
   usleep(millisecs*1000);
}

#endif // Linux or Mac OS

#include <algorithm>    // std::sort
#include "scientisst.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "../ext/rapidjson/include/rapidjson/document.h"
#include "../ext/rapidjson/include/rapidjson/writer.h"
#include "../ext/rapidjson/include/rapidjson/stringbuffer.h"
#include "tcp.h"
#include "udp.h"


/*****************************************************************************/

// CRC4 check function

static const unsigned char CRC4tab[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2};

static bool checkCRC4(const unsigned char *data, int len)
{
   unsigned char crc = 0;

   for (int i = 0; i < len-1; i++)
   {
      const unsigned char b = data[i];
      crc = CRC4tab[crc] ^ (b >> 4);
      crc = CRC4tab[crc] ^ (b & 0x0F);
   }

   // CRC for last byte
   crc = CRC4tab[crc] ^ (data[len-1] >> 4);
   crc = CRC4tab[crc];

   return (crc == (data[len-1] & 0x0F));
}

/*****************************************************************************/

// ScientISST public methods

ScientISST::VDevInfo ScientISST::find(void)
{
   VDevInfo devs;
   DevInfo  devInfo;

#ifdef _WIN32
   char     addrStr[40];
	WSADATA  m_data;

   if (WSAStartup(0x202, &m_data) != 0)	throw Exception(Exception::PORT_INITIALIZATION);

  WSAQUERYSETA querySet;
  ZeroMemory(&querySet, sizeof querySet);
  querySet.dwSize = sizeof(querySet);
  querySet.dwNameSpace = NS_BTH;
  
  HANDLE hLookup;
  DWORD flags = LUP_CONTAINERS | LUP_RETURN_ADDR | LUP_RETURN_NAME | LUP_FLUSHCACHE;
  bool tryempty = true;
  bool again;

  do
  {
	  again = false;
     if (WSALookupServiceBeginA(&querySet, flags, &hLookup) != 0)
     {
        WSACleanup();
        throw Exception(Exception::BT_ADAPTER_NOT_FOUND);
     }
  
	  while (1)
     {
        BYTE buffer[1500];
        DWORD bufferLength = sizeof(buffer);
        WSAQUERYSETA *pResults = (WSAQUERYSETA*)&buffer;
        if (WSALookupServiceNextA(hLookup, flags, &bufferLength, pResults) != 0)	break;
        if (pResults->lpszServiceInstanceName[0] == 0 && tryempty)
        {  // empty name : may happen on the first inquiry after the device was connected
           tryempty = false;   // redo the inquiry a second time only (there may be a device with a real empty name)
           again = true;
			  break;
        }

        DWORD strSiz = sizeof addrStr;
        if (WSAAddressToStringA(pResults->lpcsaBuffer->RemoteAddr.lpSockaddr, pResults->lpcsaBuffer->RemoteAddr.iSockaddrLength,
                                NULL, addrStr, &strSiz) == 0)
        {
           addrStr[strlen(addrStr)-1] = 0;   // remove trailing ')'
           devInfo.macAddr = addrStr+1;   // remove leading '('
           devInfo.name = pResults->lpszServiceInstanceName;
           devs.push_back(devInfo);
	     }
	  }

	  WSALookupServiceEnd(hLookup);
  } while (again);

  WSACleanup();

#else // Linux or Mac OS

#ifdef HASBLUETOOTH
    
    #define MAX_DEVS 255

    int dev_id = hci_get_route(NULL);
    int sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0)
      throw Exception(Exception::PORT_INITIALIZATION);

    inquiry_info ii[MAX_DEVS];
    inquiry_info *pii = ii;

    int num_rsp = hci_inquiry(dev_id, 8, MAX_DEVS, NULL, &pii, IREQ_CACHE_FLUSH);
    if(num_rsp < 0)
    {
      ::close(sock);
      throw Exception(Exception::PORT_INITIALIZATION);
    }

    for (int i = 0; i < num_rsp; i++)
    {
        char addr[19], name[248];

        ba2str(&ii[i].bdaddr, addr);
        if (hci_read_remote_name(sock, &ii[i].bdaddr, sizeof name, name, 0) >= 0)
        {
           devInfo.macAddr = addr;
           devInfo.name = name;
           devs.push_back(devInfo);        
        }
    }

    ::close(sock);
    if (pii != ii)   free(pii);
   
#else
   
   throw Exception(Exception::BT_ADAPTER_NOT_FOUND);
   
#endif // HASBLUETOOTH
   
#endif // Linux or Mac OS

    return devs;
}

/*****************************************************************************/

ScientISST::ScientISST(const char *address) : num_chs(0){
#ifdef _WIN32
   if (_memicmp(address, "COM", 3) == 0)
   {
      fd = INVALID_SOCKET;

	   char xport[40] = "\\\\.\\";   // preppend "\\.\"

	   strcat_s(xport, 40, address);

	   hCom = CreateFileA(xport,  // comm port name
					   GENERIC_READ | GENERIC_WRITE,
					   0,      // comm devices must be opened w/exclusive-access 
					   NULL,   // no security attributes 
					   OPEN_EXISTING, // comm devices must use OPEN_EXISTING 
					   0,      // not overlapped I/O 
					   NULL);  // hTemplate must be NULL for comm devices 

      if (hCom == INVALID_HANDLE_VALUE)
         throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);

      DCB dcb;
      if (!GetCommState(hCom, &dcb))
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }
      dcb.BaudRate = CBR_115200;
      dcb.fBinary = TRUE;
      dcb.fParity = FALSE;
      dcb.fOutxCtsFlow = FALSE;
      dcb.fOutxDsrFlow = FALSE;
      dcb.fDtrControl = DTR_CONTROL_DISABLE;
      dcb.fDsrSensitivity = FALSE;
      dcb.fOutX = FALSE;
      dcb.fInX = FALSE;
      dcb.fNull = FALSE;
      dcb.fRtsControl = RTS_CONTROL_ENABLE;
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
      if (!SetCommState(hCom, &dcb))
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }

	   COMMTIMEOUTS ct;
	   ct.ReadIntervalTimeout         = 0;
	   ct.ReadTotalTimeoutConstant    = 5000; // 5 s
	   ct.ReadTotalTimeoutMultiplier  = 0;
	   ct.WriteTotalTimeoutConstant   = 5000; // 5 s
	   ct.WriteTotalTimeoutMultiplier = 0;

	   if (!SetCommTimeouts(hCom, &ct)) 
	   {
		   close();
		   throw Exception(Exception::PORT_INITIALIZATION);
	   }
   }
   else // address is a Bluetooth MAC address
   {
      hCom = INVALID_HANDLE_VALUE;

      WSADATA m_data;
      if (WSAStartup(0x202, &m_data) != 0)
         throw Exception(Exception::PORT_INITIALIZATION);

      SOCKADDR_BTH so_bt;
      int siz = sizeof so_bt;
      if (WSAStringToAddressA((LPSTR)address, AF_BTH, NULL, (sockaddr*)&so_bt, &siz) != 0)
      {
         WSACleanup();
         throw Exception(Exception::INVALID_ADDRESS);
      }
      so_bt.port = 1;

      fd = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
      if (fd == INVALID_SOCKET)
      {
         WSACleanup();
         throw Exception(Exception::PORT_INITIALIZATION);
      }

      DWORD rcvbufsiz = 128*1024; // 128k
      setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*) &rcvbufsiz, sizeof rcvbufsiz);

      if (connect(fd, (const sockaddr*)&so_bt, sizeof so_bt) != 0)
      {
         int err = WSAGetLastError();
         close();

         switch(err)
         {
         case WSAENETDOWN:
            throw Exception(Exception::BT_ADAPTER_NOT_FOUND);

         case WSAETIMEDOUT:
            throw Exception(Exception::DEVICE_NOT_FOUND);

         default:
            throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
         }
      }

      readtimeout.tv_sec = 5;
      readtimeout.tv_usec = 0;
   }

#else // Linux or Mac OS

    if (memcmp(address, "/dev/", 5) == 0)
    {   
        fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0)
            throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
        
        if (fcntl(fd, F_SETFL, 0) == -1)  // remove the O_NDELAY flag
        {
            close();
            throw Exception(Exception::PORT_INITIALIZATION);
        }
    
        termios term;
        if (tcgetattr(fd, &term) != 0)
        {
            close();
            throw Exception(Exception::PORT_INITIALIZATION);
        }
    
        cfmakeraw(&term);
        term.c_oflag &= ~(OPOST);
    
        term.c_cc[VMIN] = 1;
        term.c_cc[VTIME] = 1;
    
        term.c_iflag &= ~(INPCK | PARMRK | ISTRIP | IGNCR | ICRNL | INLCR | IXON | IXOFF | IMAXBEL); // no flow control
        term.c_iflag |= (IGNPAR | IGNBRK);
    
        term.c_cflag &= ~(CRTSCTS | PARENB | CSTOPB | CSIZE); // no parity, 1 stop bit
        term.c_cflag |= (CLOCAL | CREAD | CS8);    // raw mode, 8 bits
    
        term.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOPRT | ECHOK | ECHOKE | ECHONL | ECHOCTL | ISIG | IEXTEN | TOSTOP);  // raw mode
    
        if (cfsetspeed(&term, B115200) != 0)
        {
            close();
            throw Exception(Exception::PORT_INITIALIZATION);
        }
    
        if (tcsetattr(fd, TCSANOW, &term) != 0)
        {
            close();
            throw Exception(Exception::PORT_INITIALIZATION);
        }

        //isTTY = true;

        com_mode = COM_MODE_UART;

    //Setup as an Wifi server
    }else if(memcmp(address, "server", 6) == 0){
        char *port_str;

        port_str = (char*)strrchr(address, ':')+1;  //+1 to remove the ':'
        if(port_str == NULL){
            printf("Error in reading server port number, example usage: ./scientisst server_tcp:25565\n");
            exit(-1);
        }

        //Tcp server
        if(memcmp(strrchr(address, '_')+1, "tcp", 3) == 0){
            com_mode = COM_MODE_TCP_SV;
            fd = initTcpServer(port_str);
        //Udp server
        }else if(memcmp(strrchr(address, '_')+1, "udp", 3) == 0){
            com_mode = COM_MODE_UDP;
            fd = initUdpServer(port_str, &client_addr, &client_addr_len);
        }


    }else // address is a Bluetooth MAC address
#ifdef HASBLUETOOTH
    {
        sockaddr_rc so_bt;
        so_bt.rc_family = AF_BLUETOOTH;
        if (str2ba(address, &so_bt.rc_bdaddr) < 0)
            throw Exception(Exception::INVALID_ADDRESS);
            
        so_bt.rc_channel = 1;

        fd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (fd < 0)
            throw Exception(Exception::PORT_INITIALIZATION);

        if (connect(fd, (const sockaddr*)&so_bt, sizeof so_bt) != 0)
        {
            close();
            throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
        }

        //isTTY = false;
        com_mode = COM_MODE_BT;
    }
#else
        throw Exception(Exception::PORT_COULD_NOT_BE_OPENED);
#endif // HASBLUETOOTH

#endif // Linux or Mac OS

    api_mode =  API_MODE_SCIENTISST;
    output_fd = NULL;
    bytes_to_read = 0;
}

/*****************************************************************************/

ScientISST::~ScientISST(void)
{
    try
    {
        if (num_chs != 0)  stop();
    }
    catch (Exception) {} // if stop() fails, close anyway

    close();
}

/*****************************************************************************/

void ScientISST::changeAPI(uint8_t api){
    if (num_chs != 0)   throw Exception(Exception::DEVICE_NOT_IDLE);

    api_mode = api;

    if(api <= 0 || api > 3){
        throw(Exception::INVALID_PARAMETER);
    }

    api <<= 4;
    api |= 0b11;

    send(&api, 1);
}

/*****************************************************************************/

void ScientISST::versionAndAdcChars(void){
    uint8_t cmd;
    uint8_t buff[1024];
    int rcv_bytes = 0;
    uint8_t *firmware_str;  //Pointer to beginning of firmware string
    int firmware_str_size;  //Size in bytes of firmware_str
    uint8_t *adc_chars;     //Pointer to beginning of adc chars
    int adc_chars_size;     //Size in bytes of adc_chars
    

    if (num_chs != 0)   throw Exception(Exception::DEVICE_NOT_IDLE);
        
    cmd = 0x07;
    send(&cmd, 1);    // 0  0  0  0  0  1  1  1 - Send version string

    if((rcv_bytes = recv(&buff, sizeof(buff), 1)) < 0){
        //A timeout has occurred
        throw Exception(Exception::CONTACTING_DEVICE);
    }
    firmware_str = buff;
    adc_chars = (uint8_t*)strrchr((char*)buff, '\0')+1;  //+1 to remove the '\0'
    firmware_str_size = adc_chars-firmware_str;
    adc_chars_size = rcv_bytes-firmware_str_size;
    
    //Put recieved firmware string into firmware_version
    for(int i = 0; i < firmware_str_size; i++){
        firmware_version.push_back(firmware_str[i]);
    }

    //Copy data of recieved adc chars into adc1_chars
    if(adc_chars_size != 6*sizeof(uint32_t)){
        printf("Error, recieved %dbytes, of which %dbytes are for adc_chars and was expecting %ldbytes\n", rcv_bytes, adc_chars_size, 6*sizeof(uint32_t));
        throw Exception::INVALID_PARAMETER;
    }
    
    memcpy(&adc1_chars, adc_chars, adc_chars_size);

    //Initialize fields for lookup table if necessary
    if (LUT_ENABLED && adc1_chars.atten == ADC_ATTEN_DB_11) {
        adc1_chars.low_curve = (adc1_chars.adc_num == ADC_UNIT_1) ? lut_adc1_low : lut_adc2_low;
        adc1_chars.high_curve = (adc1_chars.adc_num == ADC_UNIT_1) ? lut_adc1_high : lut_adc2_high;
    } else {
        adc1_chars.low_curve = NULL;
        adc1_chars.high_curve = NULL;
    }

    printf("ScientISST version: %s\n", firmware_version.c_str());
    printf("ScientISST Board Vref:%d\n", adc1_chars.vref);
    printf("ScientISST Board ADC Attenuation Mode:%d\n", adc1_chars.atten);

}

int ScientISST::getPacketSize(){
    uint8_t _packet_size = 0;
    int num_intern_active_chs = 0;
    int num_extern_active_chs = 0;
    rapidjson::Document d;
    char value_internal_str[50];
    char value_external_str[50];
    char aux_str[50];
    rapidjson::Value classname;
    rapidjson::Value member_name(aux_str, strlen(aux_str), d.GetAllocator());
    rapidjson::Value member_value(aux_str, strlen(aux_str), d.GetAllocator());

    for(int i = 0; i < num_chs; i++){
        //Add 24bit channel's contributuion to packet size
        if(chs[i] == AX1 || chs[i] == AX2){
            num_extern_active_chs++;
        
        //Count 12bit channels
        }else{
            num_intern_active_chs++;
        }
    } 

    if(api_mode == API_MODE_SCIENTISST){       
        //Add 24bit channel's contributuion to packet size
        _packet_size = 3*num_extern_active_chs;

        //Add 12bit channel's contributuion to packet size 
        if(!(num_intern_active_chs % 2)){                    //If it's an even number
            _packet_size += ((num_intern_active_chs*12)/8);
        }else{
            _packet_size += (((num_intern_active_chs*12)-4)/8); //-4 because 4 bits can go in the I/0 byte 
        }
        _packet_size += 2;  //for the I/Os and seq+crc bytes

    }else if(api_mode == API_MODE_JSON){
        d.SetObject();

        //Load value strings with channels' respective max values
        sprintf(value_internal_str, "%04d", 4095);     
        sprintf(value_external_str, "%08d", 16777215);

        for(int i = 0; i < num_chs; i++){
            //If it's internal ch
            if(chs[i] <= 6){
                sprintf(aux_str, "AI%d", chs[i]);
                member_name.SetString(aux_str, d.GetAllocator());
                member_value.SetString(value_internal_str, d.GetAllocator());
                d.AddMember(member_name, member_value, d.GetAllocator());
                
            }else{
                sprintf(aux_str, "AX%d", chs[i]-6);
                member_name.SetString(aux_str, d.GetAllocator());
                member_value.SetString(value_external_str, d.GetAllocator());
                d.AddMember(member_name, member_value, d.GetAllocator());
            }   
        }
        //Add IO state json objects
        d.AddMember("I1", "0", d.GetAllocator());
        d.AddMember("I2", "0", d.GetAllocator());
        d.AddMember("O1", "0", d.GetAllocator());
        d.AddMember("O2", "0", d.GetAllocator());

        // 3. Stringify the DOM
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        d.Accept(writer);

        _packet_size = strlen(buffer.GetString())+1;
    }
    return _packet_size;
}

/*****************************************************************************/

void ScientISST::start(int _sample_rate, const Vint &channels, const char* file_name, bool simulated, int api){
    uint8_t buffer[10];
    uint32_t sr;
    uint16_t cmd;
    char chMask;
    int num_frames = 0;

    sample_rate = _sample_rate;
    
    if (num_chs != 0)   throw Exception(Exception::DEVICE_NOT_IDLE);

    if(api != API_MODE_JSON && api != API_MODE_SCIENTISST){
        throw Exception(Exception::INVALID_PARAMETER);
    }

    //Clear chs vec
    memset(chs, 0, 8*sizeof(int));
    num_chs = 0;

    //Change API mode
    changeAPI(api);


    versionAndAdcChars();    // get device version string and adc characteristics

    
    //Sample rate
    sr = 0b01000011;
    sr |= _sample_rate << 8;
    send((uint8_t*)&sr, sizeof(sr));
    
    if(channels.empty()){
        chMask = 0xFF;    // all 8 analog channels
        num_chs = 8;
    }else{
        chMask = 0;
        for(Vint::const_iterator it = channels.begin(); it != channels.end(); it++){
            int ch = *it;
            chs[num_chs] = ch;        //Fill chs vector
            if (ch <= 0 || ch > 8)   throw Exception(Exception::INVALID_PARAMETER);
            const char mask = 1 << (ch-1);
            if (chMask & mask)   throw Exception(Exception::INVALID_PARAMETER);
            chMask |= mask;
            num_chs++;
        }
    }
    
    //Cleanup existing data in stream socket
    if(com_mode != COM_MODE_UDP){
        while(recv(buffer, 1) == 1);
    }
   
    //Send live mode command with channels mask
    cmd = simulated ? 0x02 : 0x01;
    cmd |= chMask << 8;
    send((uint8_t*)&cmd, sizeof(cmd));

    packet_size = getPacketSize();

    if(sample_rate > 100){
        bytes_to_read = !(MAX_BUFFER_SIZE%packet_size) ? MAX_BUFFER_SIZE-packet_size : MAX_BUFFER_SIZE-(MAX_BUFFER_SIZE%packet_size);
    }else{
        bytes_to_read = packet_size;
    }

    if(bytes_to_read % packet_size){
        printf("Error, bytes_to_read needs to be devisible by packet_size\n");
        exit(EXIT_FAILURE);
    }else{
        num_frames = bytes_to_read/packet_size;
    }

    frames.resize(num_frames);  // resize the frames vector with num_frames frames

    //Open file and write header
    initFile(file_name);
}

/*****************************************************************************/

void ScientISST::stop(void){
    uint8_t buffer[10];
    uint8_t cmd;

    if (num_chs == 0)   throw Exception(Exception::DEVICE_NOT_IN_ACQUISITION);

    cmd = 0x00;
    send(&cmd, 1); // 0  0  0  0  0  0  0  0 - Go to idle mode

    num_chs = 0;
    sample_rate = 0;

    //Cleanup existing data in bluetooth socket
    while(recv(buffer, 1) == 1);

    fclose(output_fd);
}

/*****************************************************************************/

int ScientISST::read(){
    unsigned char rcv_buff[1*1000*1000];
    unsigned char *buffer;
    int mid_frame_flag = 0;
    rapidjson::Document d;
    char memb_name[50];
    int curr_ch;
    char* junk;
    int byte_it = 0;

    if(num_chs == 0)   throw Exception(Exception::DEVICE_NOT_IN_ACQUISITION);

    if(frames.empty()){
        printf("frames is empty\n");
        return -1;
    }

    recv(rcv_buff, bytes_to_read);

    buffer = rcv_buff;
    for(VFrame::iterator it = frames.begin(); it != frames.end(); it++){
        
        
        if(!checkCRC4(buffer, packet_size)){
            //printf("checkCRC4 ERROR\n");
        }
        while (!checkCRC4(buffer, packet_size)){  // if CRC check failed, try to resynchronize with the next valid frame
            // checking with one new byte at a time
            memmove(buffer, buffer+1, packet_size-1);
            if (recv(buffer+packet_size-1, 1) != 1)    return int(it - frames.begin());   // a timeout has occurred
        }

        Frame &f = *it;

        if(api_mode == API_MODE_SCIENTISST){
            byte_it = 0;

            //Get seq number and IO states
            f.seq = buffer[packet_size-1] >> 4;
            for(int i = 0; i < 4; i++)
                f.digital[i] = ((buffer[packet_size-2] & (0x80 >> i)) != 0);

            //Get channel values
            for(int i = 0; i < num_chs; i++){
                curr_ch = chs[num_chs-1-i];

                //printf("%d\n", *(uint16_t*)(buffer+0) & 0xFFF);
                
                //If it's an AX channel
                if(curr_ch == AX1 || curr_ch == AX2){
                    f.a[curr_ch] = *(uint32_t*)(buffer+byte_it) & 0xFFFFFF;
                    byte_it += 3;

                //If it's an AI channel
                }else{
                    if(!mid_frame_flag){
                        f.a[curr_ch] = *(uint16_t*)(buffer+byte_it) & 0xFFF;
                        byte_it++;
                        mid_frame_flag = 1;
                    }else{
                        f.a[curr_ch] = *(uint16_t*)(buffer+byte_it) >> 4;
                        byte_it += 2;
                        mid_frame_flag = 0;
                    }
                }
            }
            mid_frame_flag = 0;
        }else if(api_mode == API_MODE_JSON){
            d.Parse((const char*)buffer);

            f.seq = 1;

            for(int i = 1; i < num_chs+1; i++){
                sprintf(memb_name, "AI%d", chs[i]);
                f.a[i] = strtol(d[memb_name].GetString(), &junk, 10);
            }

            f.digital[0] = strtol(d["I1"].GetString(), &junk, 10);
            f.digital[1] = strtol(d["I2"].GetString(), &junk, 10);
            f.digital[2] = strtol(d["O1"].GetString(), &junk, 10);
            f.digital[3] = strtol(d["O2"].GetString(), &junk, 10);
        }
        //printf("%d\n", f.a[0]);
        writeFrameFile(output_fd, f);
    }

    return (int) frames.size();
}

/*****************************************************************************/

void ScientISST::battery(int value){
    uint8_t cmd;

    if (num_chs != 0)   throw Exception(Exception::DEVICE_NOT_IDLE);

    if (value < 0 || value > 63)   throw Exception(Exception::INVALID_PARAMETER);   
    
    cmd = value << 2;
    send(&cmd, 1);    // <bat   threshold> 0  0 - Set battery threshold
}

/*****************************************************************************/

void ScientISST::trigger(const Vbool &digitalOutput){
   unsigned char cmd;
   const size_t len = digitalOutput.size();

   if(len != 2) throw Exception(Exception::INVALID_PARAMETER);

   cmd = 0xB3;          // 1  0  1  1  O2 O1 1  1 - Set digital outputs

    for(size_t i = 0; i < len; i++){
        if (digitalOutput[i]){
            cmd |= (0b100 << i);
        }
    }
   send(&cmd, 1);
}

/*****************************************************************************/

void ScientISST::dac(int pwmOutput){
    uint16_t cmd;

    if (pwmOutput < 0 || pwmOutput > 255)   throw Exception(Exception::INVALID_PARAMETER);

    cmd = 0xA3;             // 1  0  1  0  0  0  1  1 - Set dac output

    cmd |= pwmOutput << 8;
    send((uint8_t*)&cmd, 2);    
}

/*****************************************************************************/

ScientISST::State ScientISST::state(void){
    uint8_t cmd;
#pragma pack(1)  // byte-aligned structure

    struct StateX{
        unsigned short analog[6], battery;
        unsigned char  batThreshold, portsCRC;
    } statex;

#pragma pack()  // restore default alignment

    if (num_chs != 0)   throw Exception(Exception::DEVICE_NOT_IDLE);

    cmd = 0x0B;
    send(&cmd, 1);    // 0  0  0  0  1  0  1  1 - Send device status

    if (recv(&statex, sizeof statex) != sizeof statex)    // a timeout has occurred
        throw Exception(Exception::CONTACTING_DEVICE);

    if (!checkCRC4((unsigned char *) &statex, sizeof statex))
        throw Exception(Exception::CONTACTING_DEVICE);

    State state;

    for(int i = 0; i < 6; i++)
        state.analog[i] = statex.analog[i];

    state.battery = statex.battery;
    state.batThreshold = statex.batThreshold;

    for(int i = 0; i < 4; i++)
        state.digital[i] = ((statex.portsCRC & (0x80 >> i)) != 0);

    return state;
}

/*****************************************************************************/

const char* ScientISST::Exception::getDescription(void)
{
	switch (code)
   {
		case INVALID_ADDRESS:
			return "The specified address is invalid.";

		case BT_ADAPTER_NOT_FOUND:
			return "No Bluetooth adapter was found.";

		case DEVICE_NOT_FOUND:
			return "The device could not be found.";

		case CONTACTING_DEVICE:
			return "The computer lost communication with the device.";

		case PORT_COULD_NOT_BE_OPENED:
			return "The communication port does not exist or it is already being used.";

		case PORT_INITIALIZATION:
			return "The communication port could not be initialized.";

		case DEVICE_NOT_IDLE:
			return "The device is not idle.";
			
		case DEVICE_NOT_IN_ACQUISITION:
	        return "The device is not in acquisition mode.";
		
		case INVALID_PARAMETER:
			return "Invalid parameter.";

		case NOT_SUPPORTED:
			return "Operation not supported by the device.";

		default:
			return "Unknown error.";
	}
}

/*****************************************************************************/

void ScientISST::send(uint8_t* data, int len){
    uint8_t buff[CMD_MAX_BYTES];

    Sleep(150);

    if(len > CMD_MAX_BYTES){
        printf("Error, trying to send a command (%d bytes) bigger than max allowed (%d bytes)\n", len, CMD_MAX_BYTES);
        exit(-1);
    }

    memcpy(buff, data, len);

    //It is important to always send a fixed amount of bytes with sockets
    if(com_mode == COM_MODE_TCP_SV || com_mode == COM_MODE_TCP_CL || com_mode == COM_MODE_UDP){
        len = CMD_MAX_BYTES;
    }

#ifdef _WIN32
    if (fd == INVALID_SOCKET)
    {
        DWORD nbytwritten = 0;
        if (!WriteFile(hCom, buff, len, &nbytwritten, NULL))
            throw Exception(Exception::CONTACTING_DEVICE);

        if (nbytwritten != len)
            throw Exception(Exception::CONTACTING_DEVICE);
    }
    else{
        if (::send(fd, (char*)buff, len, 0) != len)
            throw Exception(Exception::CONTACTING_DEVICE);
    }

   
#else // Linux or Mac OS

    if(com_mode != COM_MODE_UDP){
        if(write(fd, buff, len) != len){
            throw Exception(Exception::CONTACTING_DEVICE);
        }
    }else{
        if (sendto(fd, buff, len, 0, (const struct sockaddr *) (&client_addr), client_addr_len) == -1) {
            perror("ERROR: sendto");
        }
    }

#endif
}

/*****************************************************************************/

int ScientISST::recv(void *data, int nbyttoread, uint8_t is_datagram){
    int bytes_read = 0;
#ifdef _WIN32
   if (fd == INVALID_SOCKET)
   {    
      for(int n = 0; n < nbyttoread;)
      {
         DWORD nbytread = 0;
	      if (!ReadFile(hCom, (char *) data+n, nbyttoread-n, &nbytread, NULL))
 		      throw Exception(Exception::CONTACTING_DEVICE);

         if (nbytread == 0)
         {
            DWORD stat;
            if (!GetCommModemStatus(hCom, &stat) || !(stat & MS_DSR_ON))
               throw Exception(Exception::CONTACTING_DEVICE);  // connection is lost

            return n;   // a timeout occurred
         }

         n += nbytread;
      }

	   return nbyttoread;
   }
#endif

#ifndef _WIN32 // Linux or Mac OS
   timeval  readtimeout;
   readtimeout.tv_sec = 4;
   readtimeout.tv_usec = 0;
#endif

    fd_set   readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    while(bytes_read < nbyttoread){
        int state = select(FD_SETSIZE, &readfds, NULL, NULL, &readtimeout);
        if(state < 0){
            throw Exception(Exception::CONTACTING_DEVICE);
        }

        if (state == 0){
            printf("recv: Error, a timeout occured\n");
            throw Exception(Exception::CONTACTING_DEVICE);
        }

        ssize_t ret = ::read(fd, (char *)data+bytes_read, nbyttoread-bytes_read);

        if(ret <= 0){
            printf("ScientISST did not send all bytes it was supposed to send. Recieved %d/%d (bytes)\n", bytes_read, nbyttoread);
            throw Exception(Exception::CONTACTING_DEVICE);
        }
        
        bytes_read += ret;

        //If it is a datagram, stop reading
        if(is_datagram){
            break;
        }
    }

    //return nbyttoread;
    return bytes_read;
}

/*****************************************************************************/

void ScientISST::close(void){
#ifdef _WIN32
    if (fd == INVALID_SOCKET)
        CloseHandle(hCom);
    else
    {
        closesocket(fd);
        WSACleanup();
    }
   
#else // Linux or Mac OS

    ::close(fd);

#endif
}

/*****************************************************************************/

void ScientISST::initFile(const char* file_name){
    output_fd = fopen(file_name, "w");
    if(output_fd == NULL){
        printf("Output file cannot be opened.");
        exit(-1);
    }

    fprintf(output_fd, "NSeq, I1, I2, O1, O2, ");
    for(int i = 0; i < num_chs; i++){

        if(chs[i] == AX1 || chs[i] == AX2){
            if(i == num_chs-1){
                fprintf(output_fd, "AX%d", chs[i]-6);
            }else{
                fprintf(output_fd, "AX%d, ", chs[i]-6);
            }
        }else{
            if(i == num_chs-1){
                fprintf(output_fd, "AI%d [raw], AI%d [mV]", chs[i], chs[i]);
            }else{
                fprintf(output_fd, "AI%d [raw], AI%d [mV], ", chs[i], chs[i]);
            }
        }
        
    }
    fprintf(output_fd, "\n");
}

#define VOLT_DIVIDER_FACTOR 3.399

void ScientISST::writeFrameFile(FILE* fd, Frame f){
    int channel_value_mV;

    fprintf(fd, "%d, %d, %d, %d, %d, ", f.seq, f.digital[0], f.digital[1], f.digital[2], f.digital[3]);

    for(int i = 0; i < num_chs; i++){
        if(chs[i] == AX1 || chs[i] == AX2){
            int32_t aux;
            aux = (int32_t)f.a[chs[i]] << 8;
            aux = aux >> 8;
            channel_value_mV = aux;
        }else{
            channel_value_mV = esp_adc_cal_raw_to_voltage(f.a[chs[i]], &adc1_chars)*VOLT_DIVIDER_FACTOR;
        }
        
        
        
        if(i == num_chs-1){ 
            fprintf(fd, "%d, %d", f.a[chs[i]], channel_value_mV);
        }else{
            fprintf(fd, "%d, %d, ", f.a[chs[i]], channel_value_mV);
        }
        /*if(i == num_chs-1){ 
            fprintf(fd, "%d", f.a[chs[i]]);
        }else{
            fprintf(fd, "%d, ", f.a[chs[i]]);
        }*/
    }
    fprintf(fd, "\n");
}
