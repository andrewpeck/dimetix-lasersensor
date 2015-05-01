#include <errno.h>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <cmath>     // For fabs
#include <stdlib.h>
#include "dls.hpp"
#include <termios.h> // Terminal IO

#define MAX_READ_SIZE 20
#define DEBUG       0
#define debug_print(fmt, ...) \
    do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

DLS::DLS()
{
    userCalibrated_ = false;
}

DLS::~DLS()
{
    stopTracking();
    close(fd_);
}

int DLS::serialRead (char *read_data)
{
    int i=0;
    while(i<MAX_READ_SIZE-1) {
        char tmp[] = "X";
        int n = read(fd_, tmp, 1);
        if (n==1) {
            read_data[i] = tmp [0];
            if (read_data[i]=='\n' && read_data[i-1]=='\r')
                break;
            i+=1;
        }
    }
    return EXIT_SUCCESS;
}

int DLS::serialWrite (char *write_data, int write_size)
{
    // Write Command
    write (fd_, write_data, write_size-1);
    return EXIT_SUCCESS;
}

int DLS::readTemperature () {
    char write_data[] = "s0t\r\n";

    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));
    int temperature = rxData();

    if (temperature < 0)
        printErrorMsg(temperature);

    return (temperature);
}


int DLS::getSignalQuality () {
    char write_data[] = "s0m+0\r\n";
    int len = sizeof(write_data)/sizeof(write_data[0]);
    serialWrite(write_data, len);

    int status = rxData();

    if (status < 0)
        printErrorMsg(status);

    int quality = status;
    return (quality);
}

int DLS::setMeasuringCharacteristic (int a, int b) {
    char write_data[] = "s0uc+a+b\r\n";
    sprintf(write_data,"s0uc+%1i+%1i\r\n",a,b);

    int len = sizeof(write_data)/sizeof(write_data[0]);
    serialWrite(write_data, len);

    int status = rxData();

    if (status < 0)
        printErrorMsg(status);

    status |= saveConfiguration();

    return (status);
}

int DLS::startTracking () {
    char write_data[] = "s0h\r\n";
    int len = sizeof(write_data)/sizeof(write_data[0]);
    serialWrite(write_data, len);

    int status = rxData();

    if (status < 0)
        printErrorMsg(status);

    return (status);
}

int DLS::startTrackingDelay (int delay) {
    char write_data[] = "s0h+xxx\r\n";
    sprintf(write_data,"s0h+%03i\r\n",delay);

    int len = sizeof(write_data)/sizeof(write_data[0]);
    serialWrite(write_data, len);

    int status = rxData();

    if (status < 0)
        printErrorMsg(status);

    return (status);
}

int DLS::stopTracking () {
    char write_data[] = "s0c\r\n";
    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();

    if (status < 0)
        printErrorMsg(status);

    return (status);
}

int DLS::readTracking ()
{
    int status = rxData ();
    int distance = status;

    // Failure
    if (status < 0)
        printErrorMsg(status);

    return (distance);
}


int DLS::measureDistance () {

    if (userCalibrated_) {
        char write_data[] = "s0ug\r\n";
        serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));
        debug_print("%s\n", "User calibrated");
    }
    else {
        char write_data[] = "s0g\r\n";
        serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));
    }


    int status = rxData ();
    int distance = status;

    if (status < 0)
        printErrorMsg(status);

    return distance;
}

int DLS::rxData () {
    char read_data[]  = "gNug+xxxxxxxx\r\n";

    serialRead(read_data);

    int ret = 0;

    //printf("Read data: %s", read_data);

    // Check for error indicator (@) and parse error value.
    if (read_data[2]=='@') {
        char errcode_str [] = "000";
        sprintf (errcode_str, "%*s", 3, read_data+4);
        ret = (-1) * atoi (errcode_str);
    }


    if (ret==0) {
        if (read_data[2] == '?') { // Case for Binary Yes/No returns
            ret = 1;
        }
        else {
            char data_str[] = "00000000";
            if (userCalibrated_)
                sprintf (data_str, "%*s", 8, read_data+5);
            else
                sprintf (data_str, "%*s", 8, read_data+4);
            ret = atoi (data_str);
        }
    }

    return (ret);
}

int DLS::laserOn () {
    char write_data[] = "s0o\r\n";
    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    return(status);
}


int DLS::setOutputFilter (int nsamples, int nspikes, int nerrors)
{
    // Request Current Config
    char write_data[] = "s0fi\r\n";
    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    // Read Current Config
    char read_data[] = "g0fi+aa+bb+cc\r\n";
    serialRead(read_data);
    //
    // Check for error indicator (@) and parse error value.
    if (read_data[2]=='@') {
        char errcode_str [] = "000";
        sprintf (errcode_str, "%*s", 3, read_data+4);
        return(-1 * atoi (errcode_str));
    }

    char aa_str [] = "aa";
    aa_str[0] = read_data [5];
    aa_str[1] = read_data [6];
    char bb_str [] = "bb";
    bb_str[0] = read_data [8];
    bb_str[1] = read_data [9];
    char cc_str [] = "cc";
    cc_str[0] = read_data [11];
    cc_str[1] = read_data [12];

    if ((nsamples < 0) || (nsamples > 32))
        nsamples = atoi (aa_str);
    if (nspikes < 0)
        nspikes = atoi (bb_str);
    if (nerrors < 0)
        nerrors = atoi (cc_str);

    debug_print("samples %i\n", nsamples);
    debug_print("spikes %i\n", nspikes);
    debug_print("errors %i\n", nerrors);

    if (2*nspikes+nerrors > 0.4 * nsamples) {
        fprintf(stderr, "ERROR: Make sure that (2*nspikes + nerrors) <= nsamples\n");
        nspikes = atoi (bb_str);
        nerrors = atoi (cc_str);
        nsamples = atoi (aa_str);
    }

    char write_data2[] = "s0fi+aa+bb+cc\r\n";
    sprintf(write_data2, "s0fi+%02i+%02i+%02i\r\n",nsamples,nspikes,nerrors);

    serialWrite(write_data2, sizeof(write_data2)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    return(status);
}

int DLS::laserOff () {
    char write_data[] = "s0p\r\n";
    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    return(status);
}

int DLS::setOffset(int offset)
{
    char write_data[] = "s0uof+xxxxxxxx\r\n";
    sprintf(write_data, "s0uof+%+08i\r\n", offset);
    debug_print("%s", write_data);

    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    debug_print("%i", status);

    status |= saveConfiguration();

    debug_print("%i", status);

    return(status);
}

int DLS::setGain(float gain)
{
    gain = fabs(gain);

    int gain_numer=0;
    int gain_denom=127;
    float remainder;
    float remainder_min;

    // find the best fractional representation of the given gain,
    // using the 7 bits we have available (max=127)
    for (int i=0; i<128; i++) {
        for (int j=1; j<128; j++) {
            float fraction = ((float) i) / (float (j));

            remainder = fabs(fraction-gain);

            if (i==0 && j==1)
                remainder_min=remainder;

            if (remainder < remainder_min) {
                remainder_min=remainder;

                gain_numer = i;
                gain_denom = j;
            }
        }
    }

    char write_data[] = "s0uga+xxxxxxxx+yyyyyyyy\r\n";
    sprintf(write_data, "s0uga+%08i+%08i\r\n", gain_numer, gain_denom);
    debug_print("%s\n", write_data);
    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    status |= saveConfiguration();
    return(status);
}

int DLS::saveConfiguration()
{
    char write_data[] = "s0s\r\n";

    serialWrite(write_data, sizeof(write_data)/sizeof(write_data[0]));

    int status = rxData();
    if (status < 0)
        printErrorMsg(status);

    return(status);

}


int DLS::setInterfaceAttribs (int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd_, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    //tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;     // 7-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 20;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    //tty.c_cflag &= ~(PARENB);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag |= ICANON; // Canonical Mode
    tty.c_iflag |= IGNCR; // Ignore carriage returns

    cfmakeraw (&tty);
    tcflush(fd_,TCIFLUSH);

    if (tcsetattr (fd_, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void DLS::setBlocking (int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd_, &tty) != 0)
    {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd_, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}

int DLS::setFD (int fd)
{
    fd_ = fd;
    setInterfaceAttribs (B115200, 0);   // set speed to 115,200 bps, 8n1 (no parity)
    setBlocking (0);                    // set no blocking
    return EXIT_SUCCESS;
}

int DLS::setUserCalibrated (bool enabled)
{
    userCalibrated_ = enabled;
    return EXIT_SUCCESS;
}

void DLS::printErrorMsg (int err)
{
    char msg [200] = "";
    switch(err)
    {
        case -203: sprintf(msg, "Wrong syntax in command, prohibited parameter in command entry or non-valid result"); break;
        case -210: sprintf(msg, "Not in tracking mode, start tracking mode first"); break;
        case -211: sprintf(msg, "Sampling too fast, set the sampling time to a larger value"); break;
        case -212: sprintf(msg, "Command cannot be executed because tracking mode is active, first use command sNc to stop tracking mode"); break;
        case -220: sprintf(msg, "Communication error, check configuration settings"); break;
        case -230: sprintf(msg, "Distance value overflow caused by wrong user configuration. Change user offset (and/or user gain)");                            break;
        case -231: sprintf(msg, "Wrong mode for digital input status read, activate DI1");                            break;
        case -232: sprintf(msg, "Digital output 1 cannot be set if configured as digital input");                            break;
        case -233: sprintf(msg, "Number cannot be displayed. (Check Output Format)");                            break;
        case -244: sprintf(msg, "Distance out of range.");                            break;
        case -236: sprintf(msg, "Digital output manual mode cannot be activated when configured as digital input");                            break;
        case -252: sprintf(msg, "Temperature too high (Contact Dimetix!). ");                            break;
        case -253: sprintf(msg, "Temperature too low (Contact Dimetix!)");                            break;
        case -254: sprintf(msg, "Bad signal from target, It takes too long to measure according distance. Use white surface or reflective target");                            break;
        case -255: sprintf(msg, "Received signal too weak or target lost in moving target characteristic (Use different target and distances)");                            break;
        case -256: sprintf(msg, "Received signal too strong (Use different target and distances)");                            break;
        case -258: sprintf(msg, "Power supply voltage is too high");                            break;
        case -259: sprintf(msg, "Power supply voltage is too low");                            break;
        case -260: sprintf(msg, "Distance cannot be calculated because of ambiguous targets");                            break;
        case -263: sprintf(msg, "Too much light; Use only Dimetix reflective target plate. \n In moving target characteristic, distance jump occured");                            break;
        case -264: sprintf(msg, "Too much light, measuring on reflective targets not possible.");                            break;
        case -330: sprintf(msg, "Acceleration of target too strong or distance jump (in moving target characteristic only).");                            break;
        case -331: sprintf(msg, "Over speed of target");                            break;
        case -360: sprintf(msg, "Configured measuring time is too short, set longer time or use 0");                            break;
        case -361: sprintf(msg, "Configured measuring time is too long, set shorter time");                            break;
        default:   sprintf(msg, "Uh-oh, unknown error code.");  break;
    }
    fprintf(stderr, "ERROR %i: %s\n", err, msg);
}
