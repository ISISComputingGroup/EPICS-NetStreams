/*************************************************************************\ 
* Copyright (c) 2013 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

/// @file NetStreamsInterface.cpp Implementation of #NetStreamsInterface class.
/// @author Freddie Akeroyd, STFC ISIS Facility, GB

#include <stdio.h>

//#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#ifdef _WIN32
#include <windows.h>
#include <comutil.h>
#else
#include <math.h>
#include <unistd.h>
#endif /* _WIN32 */

#include <string>
#include <vector>
#include <map>
#include <list>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <iterator>

#include <cvirte.h>		
#include <userint.h>
#include <cvinetstreams.h>

#include <shareLib.h>
#include <macLib.h>
#include <epicsGuard.h>
#include <epicsString.h>
#include <errlog.h>
#include <cantProceed.h>
#include <epicsTime.h>

#include "pugixml.hpp"

#include "asynPortDriver.h"

#include "NetStreamsInterface.h"
#include "cnsconvert.h"

#define MAX_PATH_LEN 256
static const char *driverName="NetStreamsInterface"; ///< Name of driver for use in message printing 

/// An STL exception object encapsulating a shared variable error message
class NetStreamsException : public std::runtime_error
{
public:
	explicit NetStreamsException(const std::string& message) : std::runtime_error(message) { }
	explicit NetStreamsException(const std::string& function, int code) : std::runtime_error(ni_message(function, code)) { }
private:
	static std::string ni_message(const std::string& function, int code)
	{
	    return function + ": " + CNSGetErrorDescription(code);
	}
};

#define ERROR_CHECK(__func, __code) \
    if (__code < 0) \
	{ \
	    throw NetStreamsException(__func, __code); \
	}



template <class T>
static void split_string(const std::string& str, T& cont, char delim)
{
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        cont.push_back(token);
    }
}

static CNSType getCNSType(const std::string& type)
{
		if (type == "float64" || type == "float64array")
		{
			return CNSTypeDouble;
		}
		else if (type == "int32" || type == "int32array")
		{
			return CNSTypeInt32;
		}
		else if (type == "int64" || type == "int64array")
		{
			return CNSTypeInt64;
		}
		else if (type == "boolean")
		{
			return CNSTypeBool;
		}
		else if (type == "string")
		{
			return CNSTypeString;
		}
		else if (type == "float32array")
		{
			return CNSTypeSingle;
		}
		else if (type == "int16array")
		{
			return CNSTypeInt16;
		}
		else if (type == "int8array")
		{
			return CNSTypeInt8;
        }
		else if (type == "timestamp" || type == "ftimestamp") // int64array[2]
		{
			return CNSTypeUInt64;
		}
        else
        {
			return CNSTypeForce32BitSize;
        }
}

static void createCNSData(const std::string& type, CNSData* data)
{
    size_t ndims = 1, dims[1] = {1};
    CNSData array_elem;
    int status;
    std::vector<std::string> elements;
    split_string(type, elements, ',');
    size_t nfields = elements.size();
    if (nfields > 1)
    {
        CNSData* fields = new CNSData[nfields];
        for(int i=0; i<nfields; ++i)
        {
            createCNSData(elements[i], fields + i);
        }
        status = CNSDataCreateStruct(fields, nfields, data);
        ERROR_CHECK("CNSDataCreateStruct", status);
        // Discard the intermediate CNSData objects
        for(int i=0; i<nfields; ++i)
        {
            status = CNSDataDiscard(fields[i]);
            ERROR_CHECK("CNSDataDiscard", status);
        }
    }
	else if (type == "float64")
		{
            status = CNSDataCreateScalar(CNSTypeDouble, data, static_cast<double>(0));
            ERROR_CHECK("CNSDataCreateScalar", status);
		}
		else if (type == "float64array")
		{
            createCNSData("float64", &array_elem);
            status = CNSDataCreateArray(&array_elem, dims, ndims, data);
            ERROR_CHECK("CNSDataCreateArray", status);
		}
		else if (type == "int32")
		{
            status = CNSDataCreateScalar(CNSTypeInt32, data, static_cast<int>(0));
            ERROR_CHECK("CNSDataCreateScalar", status);
		}
		else if (type == "int64")
		{
            status = CNSDataCreateScalar(CNSTypeInt64, data, static_cast<int64_t>(0));
            ERROR_CHECK("CNSDataCreateScalar", status);
		}
		else if (type == "uint64")
		{
            status = CNSDataCreateScalar(CNSTypeUInt64, data, static_cast<uint64_t>(0));
            ERROR_CHECK("CNSDataCreateScalar", status);
		}
		else if (type == "timestamp" || type == "ftimestamp" || type == "uint64array")
		{
            createCNSData("uint64", &array_elem);
            status = CNSDataCreateArray(&array_elem, dims, ndims, data);
            ERROR_CHECK("CNSDataCreateArray", status);
		}
}        


/// details about a network stream we have connected to an asyn parameter

struct NsEndpoint
{
	enum NsAccessMode { Unknown=0x0, Read=0x1, Write=0x2 };   ///< possible access modes to network shared variable
	std::string URL;   ///< full path to network shared variable 
	std::string otherURL;   ///< full path to network shared variable 
	std::string type;   ///< type as specified in the XML file e.g. float64array
	NsAccessMode access; ///< combination of #NsAccessMode

	CNSEndpoint endpointID;
	epicsTimeStamp epicsTS; ///< timestamp of shared variable update
	NsEndpoint(const std::string& type_,  NsAccessMode access_, const std::string& URL_, const std::string&  otherURL_) : URL(URL_), otherURL(otherURL_), type(type_), access(access_), endpointID(0)
	{ 
	    memset(&epicsTS, 0, sizeof(epicsTS));
	}
	/// helper for asyn driver report function
	void report(const std::string& name, FILE* fp)
	{
	    fprintf(fp, "Report for endpoint name \"%s\" type \"%s\" local endpoint \"%s\"\n", name.c_str(), type.c_str(), URL.c_str());
		char tbuffer[60];
		if (epicsTimeToStrftime(tbuffer, sizeof(tbuffer), "%Y-%m-%d %H:%M:%S.%06f", &epicsTS) <= 0)
		{
			strcpy(tbuffer, "<unknown>");
		}
		fprintf(fp, "  Update time: %s\n", tbuffer);
		int i, error;
		unsigned long long ull;
		size_t st;
		// windows doesn't have %z format for size_t hence cast to unsigned long long
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeIsEndpointConnected, &i);
		fprintf(fp, "  Connected: %s\n", ((error >= 0 && i != 0) ? "Yes" : "No"));
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeNumberOfDisconnections, &st);
		if (error >= 0) {
		    fprintf(fp, "  Number of disconnections: %llu\n", static_cast<unsigned long long>(st));
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeTotalItemsRead, &ull);
		if (error >= 0) {
		    fprintf(fp, "  Items read: %llu\n", ull);
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeTotalItemsWritten, &ull);
		if (error >= 0) {
		    fprintf(fp, "  Items written: %llu\n", ull);
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeItemsAvailForReading, &st);
		if (error >= 0) {
		    fprintf(fp, "  Items available for reading: %llu\n", static_cast<unsigned long long>(st));
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeItemsAvailForWriting, &st);
		if (error >= 0) {
		    fprintf(fp, "  Items available for writing: %llu\n", static_cast<unsigned long long>(st));
		}
	}
 
    void connect()
    {
        CNSDirection dir;
		if (access & NsEndpoint::Read)
		{
			dir = CNSDirectionReader;
		}
		else
		{
			dir = CNSDirectionWriter;			
		}
        if (otherURL.size() > 0)
        {
            std::cerr << "connect: connecting \"" << URL << "\" to \"" << otherURL << "\"" << std::endl;
        }
        else
        {
            std::cerr << "connect: waiting for connection to \"" << URL << "\"" << std::endl;
        }
        int error;
        if (type.find(',') != std::string::npos) // is it a structure?
        {
            CNSData data;
            createCNSData(type, &data);
		    error = CNSNewEndpoint(URL.c_str(), otherURL.c_str(), data, 100, 0, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewEndpoint", error);
        }
        else if (type.find("array") != std::string::npos) // is it an array?
        {
		    error = CNSNewArrayEndpoint(URL.c_str(), otherURL.c_str(), getCNSType(type), 100, 0, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewArrayEndpoint", error);
        }
        else
        {
		    error = CNSNewScalarEndpoint(URL.c_str(), otherURL.c_str(), getCNSType(type), 100, 0, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewScalarEndpoint", error);
        }
        std::cerr << "connect: connected to \"" << URL << "\" to \"" << otherURL << "\"" << std::endl;
        while(true)
        {
            read();
        }
    }
    
    void read()
    {
        int error;
        if (type.find(',') != std::string::npos) // is it a structure?
        {
            CNSData data;
            size_t nfields;
            error = CNSReadData(endpointID, CNSWaitForever, &data);
            ERROR_CHECK("CNSReadData", error);
            CNSType dtype;
            error = CNSDataGetType(data, &dtype);
            ERROR_CHECK("CNSDataGetType", error);
            if (dtype != CNSTypeStruct)
            {
                std::cerr << "error\n"; 
            }         
            error = CNSDataGetNumStructFields(data, &nfields);
            ERROR_CHECK("CNSDataGetNumStructFields", error);
            CNSData*fields = new CNSData[nfields];
            error = CNSDataGetStructFields(data, fields, nfields);
            ERROR_CHECK("CNSDataGetStructFields", error);
            for(int i=0; i<nfields; ++i)
            {
                error = CNSDataGetType(fields[i], &dtype);
            ERROR_CHECK("CNSDataGetType", error);
                if (dtype == CNSTypeArray)
                {
                    CNSType elem_type;
                    size_t numDimensions, numElements = 1;
                    size_t dimensions[10];
                    error = CNSDataGetArrayElementType(fields[i], &elem_type); 
            ERROR_CHECK("CNSDataGetArrayElementType", error);
                    error = CNSDataGetNumArrayDimensions (fields[i], &numDimensions);
            ERROR_CHECK("CNSDataGetNumArrayDimensions", error);
                    error = CNSDataGetArrayDimensions (fields[i], dimensions, numDimensions);  
            ERROR_CHECK("CNSDataGetArrayDimensions", error);
                    for(int j=0; j<numDimensions; ++j)
                    {
                        numElements *= dimensions[j];
                    }
                    CNSData* array = new CNSData[numElements];                  
                    error = CNSDataGetArrayValue(fields[i], array, numElements);
            ERROR_CHECK("CNSDataGetArrayValue", error);
                    error = CNSDataGetType(array[0], &dtype);
            ERROR_CHECK("CNSDataGetType", error);
                    if (dtype != elem_type)
                    {
                        std::cerr << "error\n"; 
                    }
                    double* dblarray = new double[numElements];
                    for(int j=0; j<numElements; ++j)
                    {                    
                        error = CNSDataGetScalarValue(array[j], dblarray + j);
                        ERROR_CHECK("CNSDataGetScalarValue", error);
                    }
                    std::cerr << "read " << numElements << " " << dblarray[0] << std::endl;
                }
                else
                {
                    std::cerr << "no data" << std::endl;
                }
            }
            for(int i=0; i<nfields; ++i)
            {
                error = CNSDataDiscard(fields[i]);
            ERROR_CHECK("CNSDataDiscard", error);
            }
        }
        else if (type.find("array") != std::string::npos)
        {
            void* array;
            size_t num_elements;
            error = CNSReadArray(endpointID, CNSWaitForever, &array, &num_elements);
            ERROR_CHECK("CNSReadArray", error);
            std::cerr << "num elements: " << num_elements << " " << *(double*)array << std::endl;
            error = CNSFreeMemory(array);
            ERROR_CHECK("CNSFreeMemory", error);
        }
        else
        {
            void* value;
            error = CNSReadScalar(endpointID, CNSWaitForever, &value);
            ERROR_CHECK("CNSReadScalar", error);
            std::cerr << "read value" << std::endl;
        }
    }

    static void connectC(void * arg)
    {
            NsEndpoint* ep = (NsEndpoint*)arg;
			try {
				ep->connect();
			}
			catch (const std::exception& ex)
			{
				std::cerr << "Error for " << ep->URL << ": " << ex.what() << std::endl;
			}
    }
};

struct NsItem
{
	enum { Read=0x1, Write=0x2 } NsAccessMode;   ///< possible access modes to network shared variable
	std::string endpointUrl;   ///< full path to network shared variable 
	std::string otherEndpointUrl;   ///< full path to network shared variable 
	std::string type;   ///< type as specified in the XML file e.g. float64array
	unsigned access; ///< combination of #NsAccessMode
	int field; ///< if we refer to a struct, this is the index of the field (starting at 0), otherwise it is -1 
	int id; ///< asyn parameter id, -1 if not assigned
	std::vector<char> array_data; ///< only used for array parameters, contains cached copy of data as this is not stored in usual asyn parameter map
	CNSEndpoint endpointID;
	CNSType nstype;
	epicsTimeStamp epicsTS; ///< timestamp of shared variable update
	
	NsItem(const char* endpointUrl_, const char*  otherEndpointUrl_, const char* type_, unsigned access_, int field_) : endpointUrl(endpointUrl_), otherEndpointUrl(otherEndpointUrl_), type(type_), access(access_), field(field_), id(-1), endpointID(0)
	{ 
	    memset(&epicsTS, 0, sizeof(epicsTS));
	}
	void report(const std::string& name, FILE* fp) { }
};

struct NetStreamsReaderData
{
	NetStreamsInterface *ns;
	std::string param;
	NetStreamsReaderData(NetStreamsInterface* ns_, const std::string& param_) : ns(ns_), param(param_) { }
};

void NetStreamsInterface::netStreamReaderC(void* arg)
{
	NetStreamsReaderData* nsd = (NetStreamsReaderData*)arg;
	NetStreamsInterface* ns = nsd->ns;
	std::string param = nsd->param;
	delete nsd;
	ns->netStreamReader(param);
}

void NetStreamsInterface::netStreamReader(const std::string& param)
{
	NsItem* item = m_params[param];
	int status, ival;
	double dval;
	char* cval;
	while(true)
	{
		try
		{
			switch(item->nstype)
			{
				case CNSTypeInt32:
					status = CNSReadScalar(item->endpointID, CNSWaitForever, &ival);
					ERROR_CHECK("CNSReadScalar", status);
					epicsTimeGetCurrent(&(item->epicsTS));
					updateParamValue(item->id, ival, &(item->epicsTS), true);
					break;

				case CNSTypeDouble:
					status = CNSReadScalar(item->endpointID, CNSWaitForever, &dval);
					ERROR_CHECK("CNSReadScalar", status);
					epicsTimeGetCurrent(&(item->epicsTS));
					updateParamValue(item->id, dval, &(item->epicsTS), true);
					break;

				case CNSTypeString:
					status = CNSReadScalar(item->endpointID, CNSWaitForever, &cval);
					ERROR_CHECK("CNSReadScalar", status);
					epicsTimeGetCurrent(&(item->epicsTS));
					updateParamValue(item->id, cval, &(item->epicsTS), true);
					status = CNSFreeMemory(cval);
					ERROR_CHECK("CNSFreeMemory", status);
					break;
					
				default:
					break;
			}
		}
		catch(const std::exception& ex)
		{
			std::cerr << "netStreamReader: " << ex.what() << std::endl;
			epicsThreadSleep(5.0);
		}
	}
}

void NetStreamsInterface::connectVars()
{
    static const char* functionName = "connectVars";
	int error;
	CNSDirection dir;
	
	for(params_t::const_iterator it=m_params.begin(); it != m_params.end(); ++it)
	{
		NsItem* item = it->second;		
		std::cerr << "connectVars: connecting to \"" << item->endpointUrl << "\"" << std::endl;
		if (item->access & NsItem::Read)
		{
			dir = CNSDirectionReader;
		}
		else
		{
			dir = CNSDirectionWriter;			
		}		
		error = CNSNewScalarEndpoint(item->endpointUrl.c_str(), item->otherEndpointUrl.c_str(), item->nstype, 100, 0, dir, CNSWaitForever, NULL, &(item->endpointID));
	    ERROR_CHECK("CNSNewScalarEndpoint", error);
		if (item->access & NsItem::Read)
		{
			NetStreamsReaderData* nsd = new NetStreamsReaderData(this, it->first); 
		    std::string thread_name = std::string("NetStreams-") + it->first;
		    if (epicsThreadCreate(thread_name.c_str(),
		              epicsThreadPriorityMedium,
		              epicsThreadGetStackSize(epicsThreadStackMedium),
		              (EPICSTHREADFUNC)netStreamReaderC, nsd) == 0)
			{
				printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
				return;
			}
		}
	}
}


template<typename T>
void NetStreamsInterface::updateParamValue(int param_index, T val, epicsTimeStamp* epicsTS, bool do_asyn_param_callbacks)
{
	const char *paramName = NULL;
	m_driver->getParamName(param_index, &paramName);
	m_driver->lock();
	m_driver->setTimeStamp(epicsTS);
	if (m_params[paramName]->type == "float64")
	{
	    m_driver->setDoubleParam(param_index, convertToScalar<double>(val));
	}
	else if (m_params[paramName]->type == "int32" || m_params[paramName]->type == "boolean")
	{
	    m_driver->setIntegerParam(param_index, convertToScalar<int>(val));
	}
	else if (m_params[paramName]->type == "string")
	{
	    m_driver->setStringParam(param_index, convertToPtr<char>(val));
	}
	else
	{
	    std::cerr << "updateParamValue: unknown type \"" << m_params[paramName]->type << "\" for param \"" << paramName << "\"" << std::endl;
	}
	if (do_asyn_param_callbacks)
	{
		m_driver->callParamCallbacks();
	}
	m_driver->unlock();
}

/// called externally with m_driver locked
template <typename T> 
void NetStreamsInterface::readArrayValue(const char* paramName, T* value, size_t nElements, size_t* nIn)
{
	std::vector<char>& array_data =  m_params[paramName]->array_data;
	size_t n = array_data.size() / sizeof(T);
	if (n > nElements)
	{
	    n = nElements;
	}
	*nIn = n;
	memcpy(value, &(array_data[0]), n * sizeof(T));
	m_driver->setTimeStamp(&(m_params[paramName]->epicsTS));
}


static epicsThreadOnceId onceId = EPICS_THREAD_ONCE_INIT;

static void initCV(void*)
{
#ifdef _WIN32
    char* dummy_argv[2] = { strdup("NetStreamsInterface"), NULL };
	if (InitCVIRTE (0, dummy_argv, 0) == 0)
		throw std::runtime_error("InitCVIRTE");
#endif
}

/// expand epics environment strings using previously saved environment  
/// based on EPICS macEnvExpand()
char* NetStreamsInterface::envExpand(const char *str)
{
    long destCapacity = 128;
    char *dest = NULL;
    int n;
    do {
        destCapacity *= 2;
        /*
         * Use free/malloc rather than realloc since there's no need to
         * keep the original contents.
         */
        free(dest);
        dest = static_cast<char*>(mallocMustSucceed(destCapacity, "NetStreamsInterface::envExpand"));
        n = macExpandString(m_mac_env, str, dest, destCapacity);
    } while (n >= (destCapacity - 1));
    if (n < 0) {
        free(dest);
        dest = NULL;
    } else {
        size_t unused = destCapacity - ++n;

        if (unused >= 20)
            dest = static_cast<char*>(realloc(dest, n));
    }
    return dest;
}

/// \param[in] configSection @copydoc initArg1
/// \param[in] configFile @copydoc initArg2
/// \param[in] options @copydoc initArg4
NetStreamsInterface::NetStreamsInterface(const char *configSection, const char* configFile, int options) : 
				m_configSection(configSection), m_options(options), m_mac_env(NULL)
{
	epicsThreadOnce(&onceId, initCV, NULL);
	// load current environment into m_mac_env, this is so we can create a macEnvExpand() equivalent 
	// but tied to the environment at a specific time. It is useful if we want to load the same 
	// XML file twice but with a macro defined differently in each case 
	if (macCreateHandle(&m_mac_env, NULL) != 0)
	{
		throw std::runtime_error("Cannot create mac handle");
	}
	for(char** cp = environ; *cp != NULL; ++cp)
	{
		char* str_tmp = strdup(*cp);
		char* equals_loc = strchr(str_tmp, '='); // split   name=value   string
		if (equals_loc != NULL)
		{
		    *equals_loc = '\0';
		    macPutValue(m_mac_env, str_tmp, equals_loc + 1);
		}
		free(str_tmp);
	}
	char* configFile_expanded = envExpand(configFile);
	m_configFile = configFile_expanded;
	epicsAtExit(epicsExitFunc, this);

    pugi::xml_parse_result result = m_xmlconfig.load_file(configFile_expanded);
	free(configFile_expanded);
	if (result)
	{
	    std::cerr << "Loaded XML config file \"" << m_configFile << "\" (expanded from \"" << configFile << "\")" << std::endl;
	}
    else
    {
		throw std::runtime_error("Cannot load XML \"" + m_configFile + "\" (expanded from \"" + std::string(configFile) + "\"): load failure: "
		    + result.description());
    }
}

// need to be careful here as might get called at wrong point. May need to check with driver.
void NetStreamsInterface::epicsExitFunc(void* arg)
{
//	NetStreamsInterface* netvarint = static_cast<NetStreamsInterface*>(arg);
//	if (netvarint == NULL)
//	{
//		return;
//	}
//	if ( netvarint->checkOption(Something) )
//	{
//	}
    CNSFinish();
}

size_t NetStreamsInterface::nParams()
{
	char control_name_xpath[MAX_PATH_LEN];
	epicsSnprintf(control_name_xpath, sizeof(control_name_xpath), "/netvar/section[@name='%s']/param", m_configSection.c_str());
	try
	{
        pugi::xpath_node_set params = m_xmlconfig.select_nodes(control_name_xpath);
        return params.size();
	}
	catch(const std::exception& ex)
	{
	    std::cerr << "nparams failed " << ex.what() << std::endl;
	    return 0;
	}
}

void NetStreamsInterface::createParams(asynPortDriver* driver)
{
    static const char* functionName = "createParams";
    m_driver = driver;
    getEndpoints();
    
    return;
    
	getParams();
	for(params_t::iterator it=m_params.begin(); it != m_params.end(); ++it)
	{
		NsItem* item = it->second;
		if (item->type == "float64")
		{
			item->nstype = CNSTypeDouble;
			m_driver->createParam(it->first.c_str(), asynParamFloat64, &(item->id));
		}
		else if (item->type == "int32" || item->type == "boolean")
		{
			item->nstype = CNSTypeInt32;
			m_driver->createParam(it->first.c_str(), asynParamInt32, &(item->id));
		}
		else if (item->type == "string")
		{
			item->nstype = CNSTypeString;
			m_driver->createParam(it->first.c_str(), asynParamOctet, &(item->id));
		}
		else if (item->type == "float64array")
		{
			item->nstype = CNSTypeDouble;
			m_driver->createParam(it->first.c_str(), asynParamFloat64Array, &(item->id));
		}
		else if (item->type == "float32array")
		{
			item->nstype = CNSTypeSingle;
			m_driver->createParam(it->first.c_str(), asynParamFloat32Array, &(item->id));
		}
		else if (item->type == "int32array")
		{
			item->nstype = CNSTypeInt32;
			m_driver->createParam(it->first.c_str(), asynParamInt32Array, &(item->id));
		}
		else if (item->type == "int16array")
		{
			item->nstype = CNSTypeInt16;
			m_driver->createParam(it->first.c_str(), asynParamInt16Array, &(item->id));
		}
		else if (item->type == "int8array")
		{
			item->nstype = CNSTypeInt8;
			m_driver->createParam(it->first.c_str(), asynParamInt8Array, &(item->id));
		}
		else
		{
			errlogSevPrintf(errlogMajor, "%s:%s: unknown type %s for parameter %s\n", driverName, 
			                functionName, item->type.c_str(), it->first.c_str());
		}
	}
	connectVars();
}

void NetStreamsInterface::getEndpoints()
{
    static const char* functionName = "getEndpoints";
	m_endpoints.clear();
	char control_name_xpath[MAX_PATH_LEN];
	epicsSnprintf(control_name_xpath, sizeof(control_name_xpath), "/netvar/section[@name='%s']/endpoint", m_configSection.c_str());
    pugi::xpath_node_set endpoints;
	try
	{
	    endpoints = m_xmlconfig.select_nodes(control_name_xpath);
	    if (endpoints.size() == 0)
	    {
	        std::cerr << "getEndpoints failed" << std::endl;
		    return;
	    }
	}
	catch(const std::exception& ex)
	{
	    std::cerr << "getEndpoints failed " << ex.what() << std::endl;
		return;
	}
	for (pugi::xpath_node_set::const_iterator it = endpoints.begin(); it != endpoints.end(); ++it)
	{
		pugi::xpath_node node = *it;	
		std::string name = node.node().attribute("name").value();
		std::string type = node.node().attribute("type").value();
		std::string access = node.node().attribute("access").value();
		std::string URL = node.node().attribute("URL").value();
		std::string otherURL = node.node().attribute("otherURL").value();
		NsEndpoint::NsAccessMode access_mode = NsEndpoint::Unknown;
		if (access == "R")
		{			
			access_mode = NsEndpoint::Read;
		}
		else if (access == "W")
		{
			access_mode = NsEndpoint::Write;
		}
		else
		{
			std::cerr << "getEndpoints: Unknown access mode \"" << access << "\" for endpoint " << name << std::endl;
            continue;
		}
		NsEndpoint* nsep = new NsEndpoint(type, access_mode, URL, otherURL);
		m_endpoints[name] = nsep;
        std::string thread_name = std::string("NetStreams-") + name;
		if (epicsThreadCreate(thread_name.c_str(),
		              epicsThreadPriorityMedium,
		              epicsThreadGetStackSize(epicsThreadStackMedium),
		              nsep->connectC, nsep) == 0)
			{
				printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
				return;
			}

	}	
}



//          <!-- params to read/write items from/to streams --> 
//          <param name="reader" type="int32" endpoint="epr1" />
//          <param name="reader_loop" type="int32" endpoint="epr2" />
//	      <param name="writer" type="int32" endpoint="epw1" /> 
//	      <param name="reader_f64a" type="float64array" endpoint="epra1" />
//		  
//		  <!-- params to read timestamp and array from structure, use timestamp to set TIME on array --> 
//	      <param name="reader_s_ts" type="ftimestamp" endpoint="epstr" field="0" /> 
//	      <param name="reader_s_f64a" type="float64array" ts_param="reader_s_ts" endpoint="epstr" field="1" /> 

void NetStreamsInterface::getParams()
{
	m_params.clear();
	char control_name_xpath[MAX_PATH_LEN];
	epicsSnprintf(control_name_xpath, sizeof(control_name_xpath), "/netvar/section[@name='%s']/param", m_configSection.c_str());
    pugi::xpath_node_set params;
	try
	{
	    params = m_xmlconfig.select_nodes(control_name_xpath);
	    if (params.size() == 0)
	    {
	        std::cerr << "getParams failed" << std::endl;
		    return;
	    }
	}
	catch(const std::exception& ex)
	{
	    std::cerr << "getParams failed " << ex.what() << std::endl;
		return;
	}
	int field;
	unsigned access_mode;
	for (pugi::xpath_node_set::const_iterator it = params.begin(); it != params.end(); ++it)
	{
		pugi::xpath_node node = *it;	
		std::string attr1 = node.node().attribute("name").value();
		std::string attr2 = node.node().attribute("type").value();
		std::string attr3 = node.node().attribute("access").value();
		std::string attr4 = node.node().attribute("endpointURL").value();
		std::string attr5 = node.node().attribute("otherEndpointURL").value();
		std::string attr6 = node.node().attribute("field").value();	
		if (attr6.size() == 0)
		{
			field = -1;
		}
		else
		{
			field = atoi(attr6.c_str());
		}
		access_mode = 0;
		if (attr3 == "R")
		{			
			access_mode = NsItem::Read;
		}
		else if (attr3 == "W")
		{
			access_mode = NsItem::Write;
		}
		else
		{
			std::cerr << "getParams: Unknown access mode \"" << attr3 << "\" for param " << attr1 << std::endl;
		}
		m_params[attr1] = new NsItem(attr4.c_str(),attr5.c_str(),attr2.c_str(),access_mode,field);	
	}	
}

template <>
void NetStreamsInterface::setValue(const char* param, const std::string& value)
{
	NsItem* item = m_params[param];
	if (C2CNS<char*>::nstype != item->nstype)
	{
        throw std::runtime_error(std::string("setValueCNV: incorrect type for param \"") + param + "\" " + C2CNS<char*>::desc + " != " + item->type);
	}		
	if (item->field != -1)
	{
        throw std::runtime_error(std::string("setValueCNV: unable to update struct variable via param \"") + param + "\"");
	}
	if (item->access & NsItem::Write)
	{
	    int status = CNSWriteScalar(item->endpointID, CNSWaitForever, value.c_str());
		ERROR_CHECK("CNSWriteScalar", status);
		status = CNSFlush(item->endpointID, CNSWaitForever, CNSFlushAllItemsAvailForRead);
		ERROR_CHECK("CNSFlush", status);
		epicsTimeGetCurrent(&(item->epicsTS));
	}
	else
	{
        throw std::runtime_error(std::string("setValueCNV: param \"")  + param + "\" does not define a writer for \"" + item->endpointUrl + "\"");
	}
}

template <typename T>
void NetStreamsInterface::setValue(const char* param, const T& value)
{
	NsItem* item = m_params[param];
	if (C2CNS<T>::nstype != item->nstype)
	{
        throw std::runtime_error(std::string("setValueCNV: incorrect type for param \"") + param + "\" " + C2CNS<T>::desc + " != " + item->type);
	}		
	if (item->field != -1)
	{
        throw std::runtime_error(std::string("setValueCNV: unable to update struct variable via param \"") + param + "\"");
	}
	if (item->access & NsItem::Write)
	{
	    int status = CNSWriteScalar(item->endpointID, CNSWaitForever, value);
		ERROR_CHECK("CNSWriteScalar", status);
		status = CNSFlush(item->endpointID, CNSWaitForever, CNSFlushAllItemsAvailForRead);
		ERROR_CHECK("CNSFlush", status);
		epicsTimeGetCurrent(&(item->epicsTS));
	}
	else
	{
        throw std::runtime_error(std::string("setValueCNV: param \"")  + param + "\" does not define a writer for \"" + item->endpointUrl + "\"");
	}
}

template <typename T>
void NetStreamsInterface::setArrayValue(const char* param, const T* value, size_t nElements)
{
	size_t dimensions[1] = { nElements };
}

/// This is called from a polling loop in the driver to 
/// update values from buffered subscribers
void NetStreamsInterface::updateValues()
{
	for(params_t::const_iterator it=m_params.begin(); it != m_params.end(); ++it)
	{
		const NsItem* item = it->second;
	}
// we used to pass false to updateParamCNV and do callParamCallbacks here
//	m_driver->lock();
//	m_driver->callParamCallbacks();
//	m_driver->unlock();
}

/// Helper for EPICS driver report function
void NetStreamsInterface::report(FILE* fp, int details)
{
	fprintf(fp, "XML ConfigFile: \"%s\"\n", m_configFile.c_str());
	fprintf(fp, "XML ConfigFile section: \"%s\"\n", m_configSection.c_str());
	fprintf(fp, "NetStreamsConfigure() Options: %d\n", m_options);
	for(params_t::iterator it=m_params.begin(); it != m_params.end(); ++it)
	{
		NsItem* item = it->second;
		item->report(it->first, fp);
	}
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template void NetStreamsInterface::setValue(const char* param, const double& value);
template void NetStreamsInterface::setValue(const char* param, const int& value);

template void NetStreamsInterface::setArrayValue(const char* param, const double* value, size_t nElements);
template void NetStreamsInterface::setArrayValue(const char* param, const float* value, size_t nElements);
template void NetStreamsInterface::setArrayValue(const char* param, const int* value, size_t nElements);
template void NetStreamsInterface::setArrayValue(const char* param, const short* value, size_t nElements);
template void NetStreamsInterface::setArrayValue(const char* param, const char* value, size_t nElements);

template void NetStreamsInterface::readArrayValue(const char* paramName, double* value, size_t nElements, size_t* nIn);
template void NetStreamsInterface::readArrayValue(const char* paramName, float* value, size_t nElements, size_t* nIn);
template void NetStreamsInterface::readArrayValue(const char* paramName, int* value, size_t nElements, size_t* nIn);
template void NetStreamsInterface::readArrayValue(const char* paramName, short* value, size_t nElements, size_t* nIn);
template void NetStreamsInterface::readArrayValue(const char* paramName, char* value, size_t nElements, size_t* nIn);

#endif /* DOXYGEN_SHOULD_SKIP_THIS */
