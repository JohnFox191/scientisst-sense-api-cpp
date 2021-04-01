
#ifndef _SCIENTISST_H
#define _SCIENTISST_H

#include <string>
#include <vector>

#ifdef _WIN32 // 32-bit or 64-bit Windows

#include <winsock2.h>

#endif

#define SIGNAL_FILENAME "performance.txt"

#define API_MODE_SCIENTISST 1
#define API_MODE_JSON 2

#define ESP_STOP_LIVE_MODE -66

#define AI1 0
#define AI2 1
#define AI3 2
#define AI4 3
#define AI5 4
#define AI6 5
#define AX1 6
#define AX2 7

/// The %BITalino device class.
class BITalino
{
public:
// Type definitions

    typedef std::vector<bool>  Vbool;   ///< Vector of bools.
    typedef std::vector<int>   Vint;    ///< Vector of ints.

    /// Information about a Bluetooth device found by BITalino::find().
    struct DevInfo
    {
        std::string macAddr; ///< MAC address of a Bluetooth device
        std::string name;    ///< Name of a Bluetooth device
    };
    typedef std::vector<DevInfo> VDevInfo; ///< Vector of DevInfo's.

    /// A frame returned by BITalino::read()
    struct Frame{
        /// %Frame sequence number (0...15).
        /// This number is incremented by 1 on each consecutive frame, and it overflows to 0 after 15 (it is a 4-bit number).
        /// This number can be used to detect if frames were dropped while transmitting data.
        char  seq;        

        /// Array of digital ports states (false for low level or true for high level).
        /// On original %BITalino, the array contents are: I1 I2 I3 I4.
        /// On %BITalino 2, the array contents are: I1 I2 O1 O2.
        bool  digital[4]; 

        
        uint32_t a[8]; /// Array of analog inputs values of the 8 channles (6 AIs and 2 AXs))
    };
    typedef std::vector<Frame> VFrame;  ///< Vector of Frame's.

    /// Current device state returned by BITalino::state()
    struct State
    {
        int   analog[6],     ///< Array of analog inputs values (0...1023)
                battery,       ///< Battery voltage value (0...1023)
                batThreshold;  ///< Low-battery LED threshold (last value set with BITalino::battery())
        /// Array of digital ports states (false for low level or true for high level).
        /// The array contents are: I1 I2 O1 O2.
        bool  digital[4];
    };

    /// %Exception class thrown from BITalino methods.
    class Exception
    {
    public:
        /// %Exception code enumeration.
        enum Code
        {
            INVALID_ADDRESS = 1,       ///< The specified address is invalid
            BT_ADAPTER_NOT_FOUND,      ///< No Bluetooth adapter was found
            DEVICE_NOT_FOUND,          ///< The device could not be found
            CONTACTING_DEVICE,         ///< The computer lost communication with the device
            PORT_COULD_NOT_BE_OPENED,  ///< The communication port does not exist or it is already being used
            PORT_INITIALIZATION,       ///< The communication port could not be initialized
            DEVICE_NOT_IDLE,           ///< The device is not idle
            DEVICE_NOT_IN_ACQUISITION, ///< The device is not in acquisition mode
            INVALID_PARAMETER,         ///< Invalid parameter
            NOT_SUPPORTED,             ///< Operation not supported by the device 
        } code;  ///< %Exception code.

        Exception(Code c) : code(c) {}      ///< Exception constructor.
        const char* getDescription(void);   ///< Returns an exception description string
    };

    // Static methods

    /** Searches for Bluetooth devices in range.
        * \return a list of found devices
        * \exception Exception (Exception::PORT_INITIALIZATION)
        * \exception Exception (Exception::BT_ADAPTER_NOT_FOUND)
        */
    static VDevInfo find(void);

    // Instance methods

    /** Connects to a %BITalino device.
        * \param[in] address The device Bluetooth MAC address ("xx:xx:xx:xx:xx:xx")
        * or a serial port ("COMx" on Windows or "/dev/..." on Linux or Mac OS X)
        * \exception Exception (Exception::PORT_COULD_NOT_BE_OPENED)
        * \exception Exception (Exception::PORT_INITIALIZATION)
        * \exception Exception (Exception::INVALID_ADDRESS)
        * \exception Exception (Exception::BT_ADAPTER_NOT_FOUND) - Windows only
        * \exception Exception (Exception::DEVICE_NOT_FOUND) - Windows only
        */
    BITalino(const char *address);
    
    /// Disconnects from a %BITalino device. If an aquisition is running, it is stopped. 
    ~BITalino();

    /** Returns the device firmware version string.
        * \remarks This method cannot be called during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IDLE)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */
    std::string version(void);
    
    /** Starts a signal acquisition from the device.
        * \param[in] samplingRate Sampling rate in Hz. Accepted values are 1, 10, 100 or 1000 Hz. Default value is 1000 Hz.
        * \param[in] channels Set of channels to acquire. Accepted channels are 0...5 for inputs A1...A6.
        * If this set is empty or if it is not given, all 6 analog channels will be acquired.
        * \param[in] simulated If true, start in simulated mode. Otherwise start in live mode. Default is to start in live mode.
        * \remarks This method cannot be called during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IDLE)
        * \exception Exception (Exception::INVALID_PARAMETER)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */
    void start(int samplingRate = 1000, const Vint &channels = Vint(), bool simulated = false);
    
    /** Stops a signal acquisition.
        * \remarks This method must be called only during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */
    void stop(void);
    
    /** Reads acquisition frames from the device.
        * This method returns when all requested frames are received from the device, or when 5-second receive timeout occurs.
        * \param[out] frames Vector of frames to be filled. If the vector is empty, it is resized to 100 frames.
        * \return Number of frames returned in frames vector. If a timeout occurred, this number is less than the frames vector size.
        * \remarks This method must be called only during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */   
    int read(VFrame &frames);
    
    /** Sets the battery voltage threshold for the low-battery LED.
        * \param[in] value Battery voltage threshold. Default value is 0.
        * Value | Voltage Threshold
        * ----- | -----------------
        *     0 |   3.4 V
        *  ...  |   ...
        *    63 |   3.8 V
        * \remarks This method cannot be called during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IDLE)
        * \exception Exception (Exception::INVALID_PARAMETER)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */
    void battery(int value = 0);
    
    /** Assigns the digital outputs states.
        * \param[in] digitalOutput Vector of booleans to assign to digital outputs, starting at first output (O1).
        * On each vector element, false sets the output to low level and true sets the output to high level.
        * If this vector is not empty, it must contain exactly 4 elements for original %BITalino (4 digital outputs)
        * or exactly 2 elements for %BITalino 2 (2 digital outputs).
        * If this parameter is not given or if the vector is empty, all digital outputs are set to low level.
        * \remarks This method must be called only during an acquisition on original %BITalino. On %BITalino 2 there is no restriction.
        * \exception Exception (Exception::DEVICE_NOT_IN_ACQUISITION)
        * \exception Exception (Exception::INVALID_PARAMETER)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        */
    void trigger(const Vbool &digitalOutput = Vbool());

    /** Assigns the analog (PWM) output value (%BITalino 2 only).
        * \param[in] pwmOutput Analog output value to set (0...255).
        * The analog output voltage is given by: V (in Volts) = 3.3 * (pwmOutput+1)/256
        * \exception Exception (Exception::INVALID_PARAMETER)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        * \exception Exception (Exception::NOT_SUPPORTED)
        */
    void dac(int pwmOutput = 100);

    /** Returns current device state (%BITalino 2 only).
        * \remarks This method cannot be called during an acquisition.
        * \exception Exception (Exception::DEVICE_NOT_IDLE)
        * \exception Exception (Exception::CONTACTING_DEVICE)
        * \exception Exception (Exception::NOT_SUPPORTED)
        */
    State state(void);

    void changeAPI(uint8_t api);

private:
    void send(uint8_t* data, int len);
    int getPacketSize();
    void close(void);
    int recv(void *data, int nbyttoread);

    int packet_size;
    int api_mode;

    int chs[8];
    int num_chs;
    bool isBitalino2;
#ifdef _WIN32
    SOCKET	fd;
    timeval  readtimeout;
    HANDLE   hCom;
#else // Linux or Mac OS
    int      fd;
    bool     isTTY;
#endif
};

#endif
