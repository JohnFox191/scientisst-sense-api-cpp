
#include "scientisst.h"
#include <stdio.h>
#include <io.h>
#include <chrono>

#ifdef _WIN32
#if defined(_MSC_VER)
typedef intptr_t ssize_t;  // Define ssize_t as a signed pointer-sized integer
#endif
#include <conio.h>

bool keypressed(void)
{
	return (_kbhit() != 0);
}

#else // Linux or Mac OS

#include <sys/select.h>
#include "../ext/rapidjson/include/rapidjson/document.h"


bool keypressed(void)
{
    fd_set   readfds;
    FD_ZERO(&readfds);
    FD_SET(0, &readfds);

    timeval  readtimeout;
    readtimeout.tv_sec = 0;
    readtimeout.tv_usec = 0;

    return (select(FD_SETSIZE, &readfds, NULL, NULL, &readtimeout) == 1);
}

#endif


int main(int argc, char **argv){
    if(argc != 3){
        printf("Arguments Error.\nExample usage: \"scientisst <device> <filename.csv>\"\n");
        return -1;
    }

    //file exists
    /*if(access(argv[1], 0) == 0){
        printf("Output file already exists. Please delete it and try again.\n");
        return -1;
    }*/

    try{
        // uncomment this block to search for Bluetooth devices (Windows and Linux)
        /*
        ScientISST::VDevInfo devs = ScientISST::find();
        for(int i = 0; i < devs.size(); i++)
            printf("%s - %s\n", devs[i].macAddr.c_str(), devs[i].name.c_str());
        return 0;
        */

        printf("Connecting to device - %s\n", argv[1]);

        ScientISST dev(argv[1]); // connect to device provided

        // use one of the lines below
        //ScientISST dev("30:AE:A4:05:62:86");  // devkit
        //ScientISST dev("AC:67:B2:1E:82:EE");  //ScientISST Sense 1.0
        //ScientISST dev("4C:11:AE:88:82:82");    //ScientISST Sense 1.1
        //ScientISST dev("AC:67:B2:1E:83:1A");    //ScientISST Sense 1.0 prof
        //08:3A:F2:49:AB:D6 nova 
        

        //ScientISST dev("COM5");  // Bluetooth virtual COM port or USB-UART COM port (Windows)

        //ScientISST dev("/dev/ttyUSB0");  // USB-UART device (Linux)
        //ScientISST dev("/dev/rfcomm0");  // Bluetooth virtual serial port (Linux)

        //ScientISST dev("/dev/tty.usbserial-A1000QIz");  // USB-UART device (Mac OS)
        //ScientISST dev("/dev/tty.scientisst-DevB");  // Bluetooth virtual serial port (Mac OS)

        puts("Connected to device. Press Enter to exit.");

        //dev.battery(10);  // set battery threshold (optional)

        //dev.trigger({true, false});                // To trigger digital outputs

        dev.start(1000, { AI1}, argv[2], false, API_MODE_SCIENTISST);

        std::chrono::steady_clock::time_point time_last_printed = std::chrono::steady_clock::now();
        do{
            dev.read();  // get multiple frames from device
            
            //print a frame every 200ms
            if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_last_printed).count() >= 5){
                const ScientISST::Frame &f = dev.frames[0];   // get a reference to the first frame of each frames block
                dev.writeFrameFile(stdout, f);
                time_last_printed = std::chrono::steady_clock::now();
            }


        }while(!keypressed());  // until a key is pressed

        dev.stop();  // stop acquisition
    } // dev is destroyed here (it goes out of scope)
    catch(ScientISST::Exception &e){
        printf("ScientISST exception: %s\n", e.getDescription());
    }

    return 0;
}
