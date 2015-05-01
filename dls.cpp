#include <getopt.h>  // Argument parsing
#include <fcntl.h>   // For file handling
#include <termios.h> // Terminal IO

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dls.hpp"

int kbhit(void);

#include <string.h>

void help();

int main(int argc, char* argv[])
{
    DLS dls;

    char portname [] = "/dev/ttyUSB0";

    int c;

    static struct option long_options[] = {
        {"port"       , required_argument , 0    , 'p'} ,
        {"user"       , no_argument       , 0    , 'u'} ,
        {"measure"    , no_argument       , 0    , 'm'} ,
        {"continuous" , optional_argument , 0    , 'c'} ,
        {"laser"      , required_argument , 0    , 'l'} ,
        {"temp"       , no_argument       , 0    , 't'} ,
        {"offset"     , required_argument , 0    , 'o'} ,
        {"speed"      , required_argument , 0    , 's'} ,
        {"gain"       , required_argument , 0    , 'g'} ,
        {"quality"    , no_argument       , 0    , 'q'} ,
        {"average"    , required_argument , 0    , 'a'} ,
        {"nspikes"    , required_argument , 0    , 'n'} ,
        {"errors"     , required_argument , 0    , 'e'} ,
        {"help"       , no_argument       , 0    , 'h'} ,
        {NULL         , 0                 , NULL ,  0 }
    };

    int option_index = 0;
    while (true) {
        c = getopt_long(argc, argv, "p:umc::l:to:g:hs:qa:n:e:", long_options, &option_index);

        if (c == (-1)) {
            break;
        }

        if ((c != 'p') && (c!='h')) {
            int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
            if (fd>0)
                dls.setFD(fd);
            else
                break;
        }

        switch (c) {
            case 0:
                printf ("option %s", long_options[option_index].name);
                if (optarg)
                    printf (" with arg %s", optarg);
                printf ("\n");
                break;
            case 'p':
                if (strcmp(portname, optarg)) {
                    sprintf(portname, "%s", optarg);
                    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
                    if (fd < 0) {
                        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
                        return 0;
                    }
                    dls.setFD(fd);
                }
            case 'u':
                dls.setUserCalibrated(true);
                break;
            case 'm':
                dls.stopTracking();
                printf("%6.1f mm\n", dls.measureDistance() / 10.0);
                break;
            case 'c':
                {
                    dls.stopTracking();

                    if (optarg != NULL) {
                        int delay_ms = atoi (optarg);
                        int delay_10ms = delay_ms / 10;

                        dls.startTrackingDelay(delay_10ms);
                    }
                    else {
                        dls.startTracking();
                    }

                    while ( !kbhit()) {
                        printf("%6.1f mm\n", dls.readTracking() / 10.0);
                    }
                    dls.stopTracking();
                    break;
                }
            case 'l':
                {
                    if (!strcmp(optarg, "on"))
                        dls.laserOn();
                    if (!strcmp(optarg, "off"))
                        dls.laserOff();
                }
                break;
            case 't':
                printf("%4.1f C\n", dls.readTemperature() / 10.0);
                break;
            case 's':
                {
                    if (!strcmp(optarg,"normal"))
                        dls.setMeasuringCharacteristic(0,0);
                    if (!strcmp(optarg,"fast"))
                        dls.setMeasuringCharacteristic(0,1);
                    if (!strcmp(optarg,"precise"))
                        dls.setMeasuringCharacteristic(0,2);
                    if (!strcmp(optarg,"natural"))
                        dls.setMeasuringCharacteristic(0,3);
                    if (!strcmp(optarg,"timed"))
                        dls.setMeasuringCharacteristic(1,1);
                    if (!strcmp(optarg,"movingerror"))
                        dls.setMeasuringCharacteristic(2,0);
                    if (!strcmp(optarg,"movingnoerror"))
                        dls.setMeasuringCharacteristic(2,1);
                    else
                        fprintf(stderr, "ERROR: Invalid Measuring Characteristic\n");
                }
                break;
            case 'o':
                {
                    float offset_mm = atof(optarg);
                    int offset = static_cast <int> (offset_mm * 10.0);
                    dls.setOffset(offset);
                }
                break;
            case 'g':
                {
                    float gain = atof(optarg);
                    dls.setGain(gain);
                }
                break;
            case 'q':
                {
                    int quality = dls.getSignalQuality();
                    printf("%i\n", quality);
                }
                break;
            case 'a':
                dls.setOutputFilter(atoi(optarg), -1, -1);
                break;
            case 'n':
                dls.setOutputFilter(-1, atoi(optarg), -1);
                break;
            case 'e':
                dls.setOutputFilter(-1, -1, atoi(optarg));
                break;
            case 'h':
                help();
                break;
            case '?':
                break;
            default:
                help();
                break;
        }
    }
    if (optind < argc) {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        printf ("\n");
    }
    return 0;
}

void help ()
{
    char usage [] = "\n usage: dls [arguments]                                            "
        "\n   e.g. dls --temp, or dls -t                                                  "
        "\n        dls --port /dev/ttyUSB1 --continuous                                   "
        "\n        dls --user --measure (same as dls -um)                                 "
        "\n        dls --laser on                                                         "
        "\n        dls --gain 0.1                                                         "
        "\n        dls --continuous=\"1000\"                                              "
        "\n                                                                               "
        "\n All units in mm or degrees C                                                  "
        "\n                                                                               "
        "\n Arguments:                                                                    "
        "\n    --measure                        Prints a single distance measurement      "
        "\n                                                                               "
        "\n    --port <tty port>                Set tty port to communication port        "
        "\n                                                                               "
        "\n    --continuous[=delay]             Continuously prints position until        "
        "\n                                     canceled by keypress                      "
        "\n                                                                               "
        "\n                                     A delay can be timed between successive   "
        "\n                                     measurements. Delays can only be set in   "
        "\n                                     increments of 10ms.                       "
        "\n                                                                               "
        "\n                                     Set to 0 blank for max sampling rate.     "
        "\n                                                                               "
        "\n                                     The sensor will complain if you set the   "
        "\n                                     sampling rate too low!                    "
        "\n                                                                               "
        "\n                                     Any key to quit !                         "
        "\n                                                                               "
        "\n    --temperature                    Outputs temperature in degrees C          "
        "\n                                                                               "
        "\n    --laser <on|off>                 Forces laser on or off for easy alignment."
        "\n                                                                               "
        "\n    --user                           Applies user calibration constants to the "
        "\n                                     output data:                              "
        "\n                                          Out = (Distance + Offset) * (Gain)   "
        "\n                                                                               "
        "\n    --offset <offset>                Set user calibration offset (in mm)       "
        "\n                                                                               "
        "\n    --gain <gain>                    Set user calibration gain (float)         "
        "\n                                                                               "
        "\n    --speed <option>                 Sets the measuring characteristic:        "
        "\n                                            normal  = (  1 mm / 10Hz )         "
        "\n                                            fast    = (0.8 mm / 20Hz )         "
        "\n                                            precise = (  1 mm / 10Hz )         "
        "\n                                            natural = (  5 mm / 0.25-6Hz)      "
        "\n                                                                               "
        "\n   --quality                         Returns signal strength as a relative     "
        "\n                                     number from 0 to 40 million               "
        "\n                                                                               "
        "\n   --average <nsamples>              Set number of samples to average          "
        "\n   --nspikes <max spikes>            Set max. number of spikes to remove       "
        "\n   --errors <max errors>             Set max. number of errors to supress      "
        "\n                                        (See manual page 16 for details)       "
        "\n                                                                               "
        "\n   --help                            Print this message.                       ";
    printf("%s\n", usage);
}
