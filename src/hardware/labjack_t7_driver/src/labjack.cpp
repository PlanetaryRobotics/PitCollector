/******************************************************************************
    * labjack.cpp -- LabJack T7 ADC class library                             *
    * May 5, 2017                                                             *
    *                                                                         *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Ford, Jordan                                                  *
    *           Papincak, Lawrence                                            *
    *                                                                         *
    * Purpose:  Provides more obvious interface functions for setting up the  *
    *           LabJack T7 to communicate measurements over TCP.              *
    *                                                                         *
    * Usage:    Include labjack.h to gain access to this library.             *
    **************************************************************************/

#include "LabJackM.h"
#include "LJM_Utilities.h"
#include "labjack.h"
#include "labjack_pinout.h"
#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

// Constructor
labjack_adc::labjack_adc(std::string ip_address, std::string serialNum) {
    _ip = ip_address;
    _sn = serialNum;
}

// Desctructor
labjack_adc::~labjack_adc(){
    _err = LJM_Close(_handle);
    ErrorCheck(_err,"LJM_Close");
}

/**
    Initializes USB connection to labjack

    @param

    @return success or failure of initialization
*/
bool labjack_adc::usbConfig() {

    // Open LabJack T7 on USB for the given serial number
    _err = LJM_Open(LJM_dtT4, LJM_ctUSB,_sn.c_str(), &_handle);
    ErrorCheck(_err,"LJM_Open");

    return true;
}

/**
    Initializes TCP connection to labjack

    @param

    @return success or failure of initialization
*/
bool labjack_adc::tcpConfig() {

    // Open LabJack T7 on ethernet for the given serial number
    _err = LJM_Open(LJM_dtT4, LJM_ctETHERNET_TCP, _ip.c_str(), &_handle); //TCP
    ErrorCheck(_err,"LJM_Open");

    return true;
}

bool labjack_adc::udpConfig() {

    // Open LabJack T7 on ethernet for the given serial number
    _err = LJM_Open(LJM_dtT4, LJM_ctETHERNET_UDP, _ip.c_str(), &_handle); //TCP
    ErrorCheck(_err,"LJM_Open");

    return true;
}

double labjack_adc::readAddress(int add, int type) {
    double ret;
    _err = LJM_eReadAddress(_handle,add,type,&ret);
    if(_err != LJME_NOERROR) {
        return -1;
    }
    return ret;
}

/**
    Reads from analog pin on LabJack
    @param  pin - string form of pin name
    @return value
*/
double labjack_adc::readAnalog(const char * pin) {
    double ret;
    _err = LJM_eReadName(_handle, pin, &ret);
    if( _err != LJME_NOERROR) {
        return -1;
    }
    return ret;
}

/**
	Writes to an analog pin on LabJack
	@param pin - string form of pin name
	@analog value - value to be written to the specified analog pin
	@return value
*/
double labjack_adc::writeAnalog(const char * pin, double analog_value){
	double ret;
	_err = LJM_eWriteName(_handle, pin, analog_value);
	if( _err != LJME_NOERROR){
		return -1;
	}
	return ret;
}

/**
    Reads from digital pin on LabJack
    @param  pin - string form of pin name
    @return value
*/
double labjack_adc::readDigital(const char * pin) {
    double ret;
    _err = LJM_eReadName(_handle, pin, &ret);
    if( _err != LJME_NOERROR) {
        return -1;
    }
    return ret;
}

/**
	Write to a digital pin on LabJack
	@param pin - string form of pin name
	@digital_value - value to be written to the specified digital pin
	@return value
*/
double labjack_adc::writeDigital(const char * pin, int digital_value){
	double ret;
	_err = LJM_eWriteName(_handle, pin, digital_value);
	if(_err != LJME_NOERROR){
		return -1;
	}
	return ret;
}
