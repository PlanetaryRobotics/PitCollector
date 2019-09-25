#ifndef LABJACK_H
#define LABJACK_H

/******************************************************************************
    * labjack.h   -- LabJack T7 ADC class header file                         *
    * May 5, 2017                                                             *
    *                                                                         *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Ford, Jordan                                                  *
    *           Papincak, Lawrence                                            *
    *                                                                         *
    * Purpose:  ADC class and register definitions.                           *
    *                                                                         *
    * Usage:    Include to gain access to LabJack T7 ADC library              *
    **************************************************************************/

#include "LabJackM.h"
#include "LJM_Utilities.h"
#include "labjack_pinout.h"
#include <stdint.h>
#include <string>

#define LABJACK_ERROR   -1
#define ON    1
#define OFF   0

class labjack_adc {

private:
	// Returned error from LJM base methods
	int _err;
	int _errAddress;
	// Labjack port handle
	int _handle;

	// Labjack specific IP and ID
	std::string _ip;
	std::string _sn;

	// Define stream variables
	int * scanList;


public:
	// Object constructor and destructor
	labjack_adc(std::string ip_address, std::string serialNum);
	~labjack_adc();

	// Methods to setup desired communication scheme with labjack
	bool tcpConfig(void);
	bool udpConfig(void);
	bool usbConfig(void);

    // Methods to read data from labjack input digital/analog pins
	double readAnalog(const char * pin);
	double writeAnalog(const char * pin, double analog_value);
	double readDigital(const char * pin);
	double writeDigital(const char * pin, int digital_value);
    double readAddress(int add, int type);
};

#endif
