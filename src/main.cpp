
#include "scientisst.h"
#include <stdio.h>
#include <unistd.h>

#ifdef _WIN32

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
    int num_frames = 0;

    if(argc != 2){
        printf("Arguments Error.\nExample usage: \"scientisst output.csv\"\n");
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

        puts("Connecting to device...");

        // use one of the lines below
        //ScientISST dev("30:AE:A4:05:62:86");  // devkit
        ScientISST dev("AC:67:B2:1E:82:EE");

        //ScientISST dev("COM5");  // Bluetooth virtual COM port or USB-UART COM port (Windows)

        //ScientISST dev("/dev/ttyUSB0");  // USB-UART device (Linux)
        //ScientISST dev("/dev/rfcomm0");  // Bluetooth virtual serial port (Linux) 

        //ScientISST dev("/dev/tty.usbserial-A1000QIz");  // USB-UART device (Mac OS)
        //ScientISST dev("/dev/tty.scientisst-DevB");  // Bluetooth virtual serial port (Mac OS) 

        puts("Connected to device. Press Enter to exit.");

        //dev.battery(10);  // set battery threshold (optional)

        dev.start(100, {AI3}, argv[1], false, API_MODE_SCIENTISST);   // start acquisition of all channels at 1000 Hz

        //dev.trigger({true, false});                // To trigger digital outputs

        if(dev.sample_rate <= 100){
            num_frames = 1;
        }else{
            num_frames = 100;
        }

        num_frames = dev.sample_rate;

        ScientISST::VFrame frames(num_frames); // initialize the frames vector with num_frames frames
        do{
            dev.read(frames); // get num_frames frames from device
            const ScientISST::Frame &f = frames[0];  // get a reference to the first frame of each num_frames frames block
            dev.writeFrameFile(stdout, f);

        }while(!keypressed()); // until a key is pressed

        dev.stop(); // stop acquisition
    } // dev is destroyed here (it goes out of scope)
    catch(ScientISST::Exception &e){
        printf("ScientISST exception: %s\n", e.getDescription());
    }

    return 0;
}
