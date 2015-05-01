
class DLS {
public:
    DLS();
    ~DLS();
    /* Returns Integer Distance Measurement in 100 micron units */
    int measureDistance ();

    /* Begin Continous Readout Mode at Highest Possible Speed */
    int startTracking();
    /* Begin Continous Readout Mode with a Delay Inserted Between Measurements */
    int startTrackingDelay(int delay);

    /* Reads out a measurement from the Continuous Readout Mode*/
    int readTracking();

    /* Stops Continuous Readout Mode */
    int stopTracking();

    /* Returns Internal Temperature in 1/10th degrees Celsius */
    int readTemperature();

    /* Sets User Calibration Offset and Gain, which can optionally be applied
     * according to the formula:
     *     Out = (Distance + Offset) * (Gain)
     * offset accepts an integer
     */
    int setOffset(int offset);
    int setGain(float gain);

    /* Turn On or Off Laser for use in Alignment */
    int laserOn();
    int laserOff();

    /* Set Filedescriptor for the serial Port */
    int setFD(int fd);

    /* Turn On or Off user calibration in the output */
    int setUserCalibrated (bool enabled);

    /*
     *    Sets the measuring characteristic of the device:
     *
     *    [a:b] = 00 Normal
     *    [a:b] = 01 Fast
     *    [a:b] = 02 Precise
     *    [a:b] = 03 Natural
     *    [a:b] = 11 Timed
     *    [a:b] = 20 Moving target characteristic with Error Freezing
     *    [a:b] = 21 Moving Target Characteristic without Error Freezing
     *
     */
    int setMeasuringCharacteristic (int a, int b);

    /*
     * Sets the characteristics of the output filter:
     *
     * The device requires that:
     *     (2*nspikes + nerrors) <= nsamples
     *
     * If values do not meet this requirement, the function will do nothing and
     * complain with an error.
     */
    int setOutputFilter(int nsamples, int nspikes, int nerrors);

    /*
     * Returns the Signal Measurement. The signal strength is returned as a
     * relative number in the typical range of 0 to 40 millions.
     * Signal strength is just an approximate value which differs from device
     * to device and also depends on environmental conditions
     */
    int getSignalQuality();

    /* Print a little help message for the command line interface */
    void help();
private:
    /* Holds the status of whether user calibrated output is turned on or off */
    bool userCalibrated_;

    /* Sets the file descriptor for the Serial Port */
    int fd_;

    /* Saves configuration to Flash */
    int saveConfiguration();

    /* Set speed and parity of the Serial Port */
    int setInterfaceAttribs (int speed, int parity);

    /* Control whether the serial port should block */
    void setBlocking (int should_block);

    /*
     * Reads data from the Dimetix Laser Sensor.. which usually comes in a
     * more-or-less common format. Decodes the serial packet and returns an
     * integer of the data that has been sent.
     *
     * Return Positive Values for Success with Data
     * Return Zero for Succeeded commands (with no data)
     * Return negative numbers for errors
     */
    int rxData();

    /* Reads a Serial Packet terminated by <cr><lf> (\r\n) */
    int serialRead (char *read_data);

    /* Writes a Serial Packet of size write_size bytes.. should be terminated
     * with <cr><lf> (\r\n) */
    int serialWrite (char *write_data, int write_size);

    /* Look-up-table of error messages from the dimetix and prints to stderr */
    void printErrorMsg(int err);
};
