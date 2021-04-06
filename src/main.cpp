
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

    if(argc != 2){
        printf("Arguments Error.\nExample usage: \"scientisst output.csv\"\n");
        return -1;
    }

    //file exists
    if(access(argv[1], 0) == 0){
        printf("Output file already exists. Please delete it and try again.\n");
        return -1;
    }

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
        ScientISST dev("30:AE:A4:05:62:86");  // device MAC address (Windows and Linux)
        
        //ScientISST dev("COM5");  // Bluetooth virtual COM port or USB-UART COM port (Windows)
        
        //ScientISST dev("/dev/ttyUSB0");  // USB-UART device (Linux)
        //ScientISST dev("/dev/rfcomm0");  // Bluetooth virtual serial port (Linux) 
        
        //ScientISST dev("/dev/tty.usbserial-A1000QIz");  // USB-UART device (Mac OS)
        //ScientISST dev("/dev/tty.scientisst-DevB");  // Bluetooth virtual serial port (Mac OS) 
        
        puts("Connected to device. Press Enter to exit.");

        std::string ver = dev.version();    // get device version string
        printf("ScientISST version: %s\n", ver.c_str());

        //dev.battery(10);  // set battery threshold (optional)



        dev.start(1000, {1, 2, 3, 4, 5, 6, 7, 8}, argv[1], false, API_MODE_JSON);   // start acquisition of all channels at 1000 Hz
        // use block below if your compiler doesn't support vector initializer lists
        /*
        ScientISST::Vint chans;
        chans.push_back(0);
        chans.push_back(1);
        chans.push_back(2);
        chans.push_back(3);
        chans.push_back(4);
        chans.push_back(5);
        dev.start(1000, chans);
        */

        //dev.trigger({true, false});                // To trigger digital outputs
        // use block below if your compiler doesn't support vector initializer lists
        /*
        ScientISST::Vbool outputs;
        outputs.push_back(false);
        outputs.push_back(false);
        outputs.push_back(true);
        outputs.push_back(false);
        dev.trigger(outputs);
        */

        ScientISST::VFrame frames(100); // initialize the frames vector with 100 frames
        do{
            dev.read(frames); // get 100 frames from device
            const ScientISST::Frame &f = frames[0];  // get a reference to the first frame of each 100 frames block
            printf("%d : %d %d %d %d ; %d %d %d %d %d %d\n",   // dump the first frame
                    f.seq,
                    f.digital[0], f.digital[1], f.digital[2], f.digital[3],
                    f.a[0], f.a[1], f.a[2], f.a[3], f.a[4], f.a[5]);

        }while(!keypressed()); // until a key is pressed

        dev.stop(); // stop acquisition
    } // dev is destroyed here (it goes out of scope)
    catch(ScientISST::Exception &e){
        printf("ScientISST exception: %s\n", e.getDescription());
    }
    
    return 0;
}
