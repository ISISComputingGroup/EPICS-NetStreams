/*************************************************************************\ 
* Copyright (c) 2013 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

/// @file NetStreamsDriver.cpp Implementation of #NetStreamsDriver class and NetStreamsConfigure() iocsh command
/// @author Freddie Akeroyd, STFC ISIS Facility, GB

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>

#include <shareLib.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <errlog.h>
#include <iocsh.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "convertToString.h"
#include "NetStreamsInterface.h"
#include "NetStreamsDriver.h"

#include <epicsExport.h>

static const char *driverName="NetStreamsDriver"; ///< Name of driver for use in message printing 

/// write a value to the driver
/// @tparam T data type of \a value
/// @param[in] pasynUser pointer to AsynUser instance
/// @param[in] functionName Name of overloaded ASYN driver function that called us, used for diagnostics
/// @param[in] value Value to write
template<typename T>
asynStatus NetStreamsDriver::writeValue(asynUser *pasynUser, const char* functionName, T value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName = NULL;
	getParamName(function, &paramName);
	try
	{
		if (m_netvarint == NULL)
		{
			throw std::runtime_error("m_netvarint is NULL");
		}
		m_netvarint->setValue(paramName, value);
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s: function=%d, name=%s, value=%s\n", 
			driverName, functionName, function, paramName, convertToString(value).c_str());
		return asynSuccess;
	}
	catch(const std::exception& ex)
	{
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
			"%s:%s: status=%d, function=%d, name=%s, value=%s, error=%s", 
			driverName, functionName, status, function, paramName, convertToString(value).c_str(), ex.what());
		return asynError;
	}
}

/// write an array to the driver
/// @tparam T Data type of \a value
/// @param[in] pasynUser pointer to AsynUser instance
/// @param[in] functionName Name of overloaded ASYN driver function that called us, used for diagnostics
/// @param[in] value Value to write
/// @param[in] nElements number of array elements
template<typename T>
asynStatus NetStreamsDriver::writeArrayValue(asynUser *pasynUser, const char* functionName, T* value, size_t nElements)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName = NULL;
	getParamName(function, &paramName);
	try
	{
		if (m_netvarint == NULL)
		{
			throw std::runtime_error("m_netvarint is NULL");
		}
		m_netvarint->setArrayValue(paramName, value, nElements);
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s: function=%d, name=%s, nElements=%d\n", 
			driverName, functionName, function, paramName, (int)nElements);
		return asynSuccess;
	}
	catch(const std::exception& ex)
	{
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
			"%s:%s: status=%d, function=%d, name=%s, nElements=%d, error=%s", 
			driverName, functionName, status, function, paramName, (int)nElements, ex.what());
		return asynError;
	}
}

/// write a float to the driver
/// @param[in] pasynUser pointer to AsynUser instance
/// @param[in] value Value to write
asynStatus NetStreamsDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	asynStatus status = writeValue(pasynUser, "writeFloat64", value);
	return (status == asynSuccess ? asynPortDriver::writeFloat64(pasynUser, value) : status);
}

asynStatus NetStreamsDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	asynStatus status = writeValue(pasynUser, "writeInt32", value);
	return (status == asynSuccess ? asynPortDriver::writeInt32(pasynUser, value) : status);
}

asynStatus NetStreamsDriver::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName = NULL;
	getParamName(function, &paramName);
	const char* functionName = "writeOctet";
	std::string value_s(value, maxChars);
	try
	{
		if (m_netvarint == NULL)
		{
			throw std::runtime_error("m_netvarint is NULL");
		}
		m_netvarint->setValue(paramName, value_s);
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s: function=%d, name=%s, value=%s\n", 
			driverName, functionName, function, paramName, value_s.c_str());
		*nActual = value_s.size();
		return asynPortDriver::writeOctet(pasynUser, value_s.c_str(), maxChars, nActual);
	}
	catch(const std::exception& ex)
	{
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
			"%s:%s: status=%d, function=%d, name=%s, value=%s, error=%s", 
			driverName, functionName, status, function, paramName, value_s.c_str(), ex.what());
		*nActual = 0;
		return asynError;
	}
}

asynStatus NetStreamsDriver::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
	return readArrayValue(pasynUser, "readFloat64Array", value, nElements, nIn);
}

asynStatus NetStreamsDriver::readFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements, size_t *nIn)
{
	return readArrayValue(pasynUser, "readFloat32Array", value, nElements, nIn);
}

asynStatus NetStreamsDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn)
{
	return readArrayValue(pasynUser, "readInt32Array", value, nElements, nIn);
}

asynStatus NetStreamsDriver::readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn)
{
	return readArrayValue(pasynUser, "readInt16Array", value, nElements, nIn);
}

asynStatus NetStreamsDriver::readInt8Array(asynUser *pasynUser, epicsInt8 *value, size_t nElements, size_t *nIn)
{
	return readArrayValue(pasynUser, "readInt8Array", value, nElements, nIn);
}

template<typename T>
asynStatus NetStreamsDriver::readArrayValue(asynUser *pasynUser, const char* functionName, T *value, size_t nElements, size_t *nIn)
{
	epicsTimeStamp epicsTS;
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName = NULL;
	getParamName(function, &paramName);
	try
	{
		if (m_netvarint == NULL)
		{
			throw std::runtime_error("m_netvarint is NULL");
		}
		m_netvarint->readArrayValue(paramName, value, nElements, nIn); // this will also update driver timestamp
		getTimeStamp(&epicsTS);
		pasynUser->timestamp = epicsTS;
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s: function=%d, name=%s, size=%d\n", 
			driverName, functionName, function, paramName, (int)nElements);
		return asynSuccess;
	}
	catch(const std::exception& ex)
	{
		*nIn = 0;
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
			"%s:%s: status=%d, function=%d, name=%s, size=%d, error=%s", 
			driverName, functionName, status, function, paramName, (int)nElements, ex.what());
		return asynError;
	}
}

asynStatus NetStreamsDriver::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    return writeArrayValue(pasynUser, "writeInt32Array", value, nElements);
}

asynStatus NetStreamsDriver::writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements)
{
    return writeArrayValue(pasynUser, "writeInt16Array", value, nElements);
}

asynStatus NetStreamsDriver::writeInt8Array(asynUser *pasynUser, epicsInt8 *value, size_t nElements)
{
    return writeArrayValue(pasynUser, "writeInt8Array", value, nElements);
}

asynStatus NetStreamsDriver::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    return writeArrayValue(pasynUser, "writeFloat64Array", value, nElements);
}

asynStatus NetStreamsDriver::writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements)
{
    return writeArrayValue(pasynUser, "writeFloat32Array", value, nElements);
}

/// EPICS driver report function for iocsh dbior command
void NetStreamsDriver::report(FILE* fp, int details)
{
	if (m_netvarint != NULL)
	{
		m_netvarint->report(fp, details);
	}
	else
	{
		fprintf(fp, "NetVarInt pointer is NULL\n");
	}
	asynPortDriver::report(fp, details);
}


/// Constructor for the #NetStreamsDriver class.
/// Calls constructor for the asynPortDriver base class and sets up driver parameters.
///
/// @param[in] netvarint  interface pointer created by NetStreamsConfigure()
/// @param[in] poll_ms  @copydoc initArg0
/// @param[in] portName @copydoc initArg3
NetStreamsDriver::NetStreamsDriver(NetStreamsInterface* netvarint, int poll_ms, const char *portName) 
	: asynPortDriver(portName, 
	0, /* maxAddr */ 
	static_cast<int>(netvarint->nParams()),
	asynInt32Mask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
	asynInt32Mask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
	ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
	1, /* Autoconnect */
	0, /* Default priority */
	0),	/* Default stack size*/
	m_netvarint(netvarint), m_poll_ms(poll_ms), m_shutting_down(false)
{
	const char *functionName = "NetStreamsDriver";
	
	m_netvarint->createParams(this);
	if (poll_ms == 0)
	{
	    std::cerr << "Warning: driver is not polling for buffered reads, only subscribers will see changes" << std::endl;
	}
    epicsAtExit(epicsExitFunc, this);

	// Create the thread for background tasks (not used at present, could be used for I/O intr scanning) 
	if (epicsThreadCreate("NetStreamsDriverTask",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)NetStreamsTask, this) == 0)
	{
		printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
		return;
	}
}

void NetStreamsDriver::epicsExitFunc(void* arg)
{
	NetStreamsDriver* driver = static_cast<NetStreamsDriver*>(arg);
	if (driver == NULL)
	{
		return;
	}
	driver->shuttingDown(true);
}


void NetStreamsDriver::NetStreamsTask(void* arg) 
{ 
	NetStreamsDriver* driver = (NetStreamsDriver*)arg; 	
	int poll_ms = driver->pollTime();
	if (poll_ms > 0)
	{
		while(!driver->shuttingDown())
		{
			driver->updateValues();
			epicsThreadSleep(static_cast<double>(poll_ms) / 1000.0);
		}
	}
}

extern "C" {

	/// EPICS iocsh callable function to call constructor of NetStreamsInterface().
	/// The function is registered via NetStreamsRegister().
	///
	/// @param[in] portName @copydoc initArg0
	/// @param[in] configSection @copydoc initArg1
	/// @param[in] configFile @copydoc initArg2
	/// @param[in] pollPeriod @copydoc initArg3
	/// @param[in] options @copydoc initArg4
	int NetStreamsConfigure(const char *portName, const char* configSection, const char *configFile, int pollPeriod, int options)
	{
		try
		{
			NetStreamsInterface* netvarint = new NetStreamsInterface(configSection, configFile, options);
			if (netvarint != NULL)
			{
				new NetStreamsDriver(netvarint, pollPeriod, portName);
				return(asynSuccess);
			}
			else
			{
				errlogSevPrintf(errlogFatal, "NetStreamsConfigure failed (NULL)\n");
				return(asynError);
			}

		}
		catch(const std::exception& ex)
		{
			errlogSevPrintf(errlogFatal, "NetStreamsConfigure failed: %s\n", ex.what());
			return(asynError);
		}
	}

	// EPICS iocsh shell commands 

	static const iocshArg initArg0 = { "portName", iocshArgString};			///< The name of the asyn driver port we will create
	static const iocshArg initArg1 = { "configSection", iocshArgString};	///< section name of \a configFile to use to configure this asyn port
	static const iocshArg initArg2 = { "configFile", iocshArgString};		///< Path to the XML input file to load configuration information from
	static const iocshArg initArg3 = { "pollPeriod", iocshArgInt};			///< poll period (ms) for BufferedReaders
	static const iocshArg initArg4 = { "options", iocshArgInt};			    ///< options as per #NetStreamsOptions enum

	static const iocshArg * const initArgs[] = { &initArg0,
		&initArg1,
		&initArg2,
		&initArg3,
		&initArg4 };

	static const iocshFuncDef initFuncDef = {"NetStreamsConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

	static void initCallFunc(const iocshArgBuf *args)
	{
		NetStreamsConfigure(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival);
	}
	
	/// Register new commands with EPICS IOC shell
	static void NetStreamsRegister(void)
	{
		iocshRegister(&initFuncDef, initCallFunc);
	}

	epicsExportRegistrar(NetStreamsRegister);

}

