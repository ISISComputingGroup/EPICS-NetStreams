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
#define NOMINMAX
#include <windows.h>
#include <comutil.h>
#else
#include <math.h>
#include <unistd.h>
#endif /* _WIN32 */

#include <sys/timeb.h>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <limits>

#if defined(_WIN32) && defined(_MSC_VER) && _MSC_VER < 1700 /* Pre VS2012 */
// boost atomic is not header only, volatile should be enough here 
// as we only write from a single thread adn also only care about
// 64bit architectures
typedef volatile uint32_t my_atomic_uint32_t;
typedef volatile uint64_t my_atomic_uint64_t;
#else
#include <atomic>
typedef std::atomic<uint32_t> my_atomic_uint32_t;
typedef std::atomic<uint64_t> my_atomic_uint64_t;
#endif

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
#include <alarm.h>

#include "pugixml.hpp"

#include "asynPortDriver.h"

#include <epicsExport.h>

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

union ScalarValue
{
    char* c;
    double d;
    int i;
};

template <class T>
static void split_string(const std::string& str, T& cont, char delim)
{
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        cont.push_back(token);
    }
}

static size_t getCNSSize(CNSType type)
{
    switch(type)
    {
        case CNSTypeDouble:
            return sizeof(CNS2C<CNSTypeDouble>::ctype);        
        case CNSTypeInt32:
            return sizeof(CNS2C<CNSTypeInt32>::ctype);        
        case CNSTypeUInt64:
            return sizeof(CNS2C<CNSTypeUInt64>::ctype);        
        case CNSTypeInt64:
            return sizeof(CNS2C<CNSTypeInt64>::ctype);        
        case CNSTypeSingle:
            return sizeof(CNS2C<CNSTypeSingle>::ctype);        
        default:
            return 0;
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
		else if (type == "uint64" || type == "uint64array")
		{
			return CNSTypeUInt64;
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
		else if (type == "timestamp" || type == "ftimestamp") // uint64array[2]
		{
			return CNSTypeUInt64;
		}
        else
        {
			throw NetStreamsException(std::string("getCNSType: unknwon type: ") + type);
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
        delete[] fields;
    }
		else if (type == "float64array")
		{
            createCNSData("float64", &array_elem);
            status = CNSDataCreateArray(&array_elem, dims, ndims, data);
            ERROR_CHECK("CNSDataCreateArray", status);
            status = CNSDataDiscard(array_elem);
            ERROR_CHECK("CNSDataDiscard", status);
		}
		else if (type == "float32array")
		{
            createCNSData("float32", &array_elem);
            status = CNSDataCreateArray(&array_elem, dims, ndims, data);
            ERROR_CHECK("CNSDataCreateArray", status);
            status = CNSDataDiscard(array_elem);
            ERROR_CHECK("CNSDataDiscard", status);
		}
		else if (type == "int32array")
		{
            createCNSData("int32", &array_elem);
            status = CNSDataCreateArray(&array_elem, dims, ndims, data);
            ERROR_CHECK("CNSDataCreateArray", status);
            status = CNSDataDiscard(array_elem);
            ERROR_CHECK("CNSDataDiscard", status);
		}
	    else if (type == "float64")
		{
            status = CNSDataCreateScalar(CNSTypeDouble, data, static_cast<double>(0));
            ERROR_CHECK("CNSDataCreateScalar", status);
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
            status = CNSDataDiscard(array_elem);
            ERROR_CHECK("CNSDataDiscard", status);
		}
		else if (type == "string")
		{
            status = CNSDataCreateScalar(CNSTypeString, data, "");
            ERROR_CHECK("CNSDataCreateScalar", status);
		}
        else
        {
			throw NetStreamsException(std::string("createCNSData: unknown type: ") + type);
        }
}        

struct NsItem
{
	std::string type;   ///< type as specified in the XML file e.g. float64array
	int field; ///< if we refer to a struct, this is the index of the field (starting at 0), otherwise it is -1 
	int id; ///< asyn parameter id, -1 if not assigned
	std::vector<char> array_data; ///< only used for array parameters, contains cached copy of data as this is not stored in usual asyn parameter map
	CNSType nstype;
    std::string endpoint;
	epicsTimeStamp epicsTS; ///< timestamp of variable update
    std::string ts_param; ///< parameter that is timestamp source
    bool with_ts;
    NetStreamsInterface* m_inf;
	
	NsItem(NetStreamsInterface* inf_, const std::string& endpoint_, const std::string& type_, int field_, const std::string& ts_param_, bool with_ts_) : 
       endpoint(endpoint_), type(type_), field(field_), id(-1), ts_param(ts_param_), with_ts(with_ts_), m_inf(inf_)
	{ 
	    memset(&epicsTS, 0, sizeof(epicsTS));
	}
	void report(const std::string& name, FILE* fp) { }
    void callback(epicsTimeStamp* epicsTSv, CNSType ep_type, void* value, size_t numElements)
    {
        // the update time for an item in a shared variable structure/cluster is the upadate time of the structure variable
        // so we need to propagate the structure time when we recurse into its fields
        epicsTSv = m_inf->getLinkedTimestamp(id, epicsTSv);
        epicsTS = *epicsTSv;
        switch(ep_type)
        {
				case CNSTypeInt32:
                    if (numElements == 1)
                    {
			            m_inf->updateParamValue(id, *(int*)value, epicsTSv, true);
                    }
                    else
                    {
			            m_inf->updateParamArrayValue(id, (int*)value, numElements, epicsTSv, true, with_ts);
                    }                        
                    break;
				case CNSTypeUInt64:
                    if (numElements == 1)
                    {
			            m_inf->updateParamValue(id, *(uint64_t*)value, epicsTSv, true);
                    }
                    else
                    {
			            m_inf->updateParamArrayValue(id, (uint64_t*)value, numElements, epicsTSv, true, with_ts);
                    }                        
                    break;
				case CNSTypeDouble:
                    if (numElements == 1)
                    {
			            m_inf->updateParamValue(id, *(double*)value, epicsTSv, true);
                    }
                    else
                    {
			            m_inf->updateParamArrayValue(id, (double*)value, numElements, epicsTSv, true, with_ts);
                    }
                    break;
				case CNSTypeSingle:
                    if (numElements == 1)
                    {
			            m_inf->updateParamValue(id, *(float*)value, epicsTSv, true);
                    }
                    else
                    {
			            m_inf->updateParamArrayValue(id, (float*)value, numElements, epicsTSv, true, with_ts);
                    }
                    break;
				case CNSTypeString:
			        m_inf->updateParamValue(id, *(const char*)value, epicsTSv, true);
                    break;
                default:
			        throw NetStreamsException(std::string("callback: unknown endpoint type: "));
                    break;
        }
    }
};

/// details about a network stream we have connected to an asyn parameter


struct NsEndpoint
{
	enum NsAccessMode { Unknown=0x0, Read=0x1, Write=0x2 };   ///< possible access modes to network shared variable
	std::string URL;   ///< full path to network shared variable 
	std::string otherURL;   ///< full path to network shared variable 
	std::string type;   ///< type as specified in the XML file e.g. float64array
	NsAccessMode access; ///< combination of #NsAccessMode
    int delay_ms;
    
    std::vector<NsItem*> m_callbacks;

	CNSEndpoint endpointID;
    CNSType endpointType;
    
    my_atomic_uint32_t m_items_read;
    my_atomic_uint64_t m_bytes_read;
    uint32_t m_last_items_read;
    uint64_t m_last_bytes_read;
    struct timeb m_last_report;
    std::list<int> field_ids;
    size_t m_bufferItems;
    size_t m_bufferAlloc;
    
	epicsTimeStamp epicsTS; ///< timestamp of endpoint update
	NsEndpoint(const std::string& type_,  NsAccessMode access_, const std::string& URL_, const std::string&  otherURL_, int delay_ms_, size_t bufferItems_, size_t bufferAlloc_) : URL(URL_), otherURL(otherURL_), type(type_), access(access_), endpointID(0), delay_ms(delay_ms_), m_bufferItems(bufferItems_), m_bufferAlloc(bufferAlloc_), m_items_read(0), m_bytes_read(0), m_last_items_read(0), m_last_bytes_read(0)
	{ 
	    memset(&epicsTS, 0, sizeof(epicsTS));
        ftime(&m_last_report);
        // we want to process timestamp fileds in a structure first, so their value is available to other members 
        std::vector<std::string> elements;
        split_string(type, elements, ',');
        for(int i=0; i<elements.size(); ++i)
        {
            if (elements[i] == "timestamp" || elements[i] == "ftimestamp" || elements[i] == "uint64array")
            {
                field_ids.push_front(i);
            }
            else
            {
                field_ids.push_back(i);
            }
        }            
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
		    fprintf(fp, "  Items read (session): %llu\n", ull);
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeTotalItemsWritten, &ull);
		if (error >= 0) {
		    fprintf(fp, "  Items written (session): %llu\n", ull);
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeItemsAvailForReading, &st);
		if (error >= 0) {
		    fprintf(fp, "  Items available for reading: %llu\n", static_cast<unsigned long long>(st));
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeItemsAvailForWriting, &st);
		if (error >= 0) {
		    fprintf(fp, "  Items available for writing: %llu\n", static_cast<unsigned long long>(st));
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeRemoteBufferSize, &st);
		if (error >= 0) {
		    fprintf(fp, "  Remote buffer size (items): %llu\n", static_cast<unsigned long long>(st));
		}
		error = CNSGetEndpointAttribute(endpointID, CNSAttributeRemoteBufferFreeSpace, &st);
		if (error >= 0) {
		    fprintf(fp, "  Remote buffer free space (items): %llu\n", static_cast<unsigned long long>(st));
		}
        struct timeb now;
		fprintf(fp, "  Items read (total): %llu\n", static_cast<unsigned long long>(m_items_read));
		fprintf(fp, "  Bytes read (total): %llu\n", static_cast<unsigned long long>(m_bytes_read));
        ftime(&now);
        double tdiff = difftime(now.time, m_last_report.time) + ((int)now.millitm - (int)m_last_report.millitm) / 1000.0;
        fprintf(fp, "  * Data rates are average since last call to this command *\n");
        fprintf(fp, "  Items read /s: %f\n", (m_items_read - m_last_items_read) / tdiff );
        fprintf(fp, "  Bytes read /s: %f\n", (m_bytes_read - m_last_bytes_read) / tdiff );
        m_last_items_read = m_items_read;
        m_last_bytes_read = m_bytes_read;
        m_last_report = now;
	}
 
    void monitorAndReconnect()
    {
        int is_connected, status;
        while(true)
        {
            try
            {
                connect();
                do {
                    epicsThreadSleep(1.0);
                    status = CNSGetEndpointAttribute(endpointID, CNSAttributeIsEndpointConnected, &is_connected);
                } while (status == 0 && is_connected == 1);
            }
 			catch (const std::exception& ex)
			{
				std::cerr << "Error for " << URL << ": " << ex.what() << std::endl;
			}        
            CNSDiscardEndpoint(endpointID);
            epicsThreadSleep(5.0);
        }
    }
    
    void connectAndRead()
    {
        while(true)
        {
            try
            {
                connect();
                while(true)
                {
                    read();
                    if (delay_ms > 0)
                    {
                        epicsThreadSleep(delay_ms);
                    }
                }
            }
 			catch (const std::exception& ex)
			{
				std::cerr << "Error for " << URL << ": " << ex.what() << std::endl;
			}
            CNSDiscardEndpoint(endpointID);
            epicsThreadSleep(5.0);
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
        if (field_ids.size() > 1) // is it a structure?
        {
            CNSData data;
            createCNSData(type, &data);
		    error = CNSNewEndpoint(URL.c_str(), otherURL.c_str(), data, m_bufferItems, m_bufferAlloc, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewEndpoint", error);
            error = CNSDataDiscard(data);
            ERROR_CHECK("CNSDataDiscard", error);
            endpointType = CNSTypeStruct;
        }
        else if (type.find("array") != std::string::npos) // is it an array?
        {
		    error = CNSNewArrayEndpoint(URL.c_str(), otherURL.c_str(), getCNSType(type), m_bufferItems, m_bufferAlloc, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewArrayEndpoint", error);
            endpointType = getCNSType(type);
        }
        else
        {
		    error = CNSNewScalarEndpoint(URL.c_str(), otherURL.c_str(), getCNSType(type), m_bufferItems, m_bufferAlloc, dir, CNSWaitForever, NULL, &endpointID);
            ERROR_CHECK("CNSNewScalarEndpoint", error);
            endpointType = getCNSType(type);
       }
       std::cerr << "connect: connected \"" << URL << "\" to \"" << otherURL << "\"" << std::endl;
    }

    void doCallbacks(CNSType ep_type, void* value, size_t numElements, int field = -1)
    {    
        for(int i=0; i<m_callbacks.size(); ++i)
        {
            if (field == m_callbacks[i]->field)
            {
                m_callbacks[i]->callback(&epicsTS, ep_type, value, numElements);
            }
        }
    }
    
    void read()
    {
        int error;
        if (field_ids.size() > 1) // is it a structure?
        {
            CNSData data;
            size_t nfields;
            error = CNSReadData(endpointID, CNSWaitForever, &data);
            ERROR_CHECK("CNSReadData", error);
            epicsTimeGetCurrent(&epicsTS);
            CNSType dtype;
            error = CNSDataGetType(data, &dtype);
            ERROR_CHECK("CNSDataGetType", error);
            if (dtype != CNSTypeStruct)
            {
                throw NetStreamsException("Invalid data type - not struct");
            }         
            error = CNSDataGetNumStructFields(data, &nfields);
            ERROR_CHECK("CNSDataGetNumStructFields", error);
            CNSData* fields = new CNSData[nfields];
            error = CNSDataGetStructFields(data, fields, nfields);
            ERROR_CHECK("CNSDataGetStructFields", error);
            for(std::list<int>::const_iterator it = field_ids.begin(); it != field_ids.end(); ++it)
            {
                int i = *it;
                if (i >= nfields)
                {
                    throw NetStreamsException(std::string("read: too few fields: "));
                }
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
                        throw NetStreamsException("Array data type mismatch");
                    }
                    size_t elem_size = getCNSSize(elem_type);
                    double* array_data = new double[numElements * elem_size / sizeof(double) + 1]; // make sure always double aligned
                    for(int j=0; j<numElements; ++j)
                    {                    
                        error = CNSDataGetScalarValue(array[j], reinterpret_cast<char*>(array_data) + j * elem_size);
                        ERROR_CHECK("CNSDataGetScalarValue", error);
                        error = CNSDataDiscard(array[j]);
                        ERROR_CHECK("CNSDataDiscard", error);
                    }
                    delete[] array;
                    doCallbacks(elem_type, array_data, numElements, i);
                    delete[] array_data;
                    m_bytes_read += (numElements * elem_size);
                }
                else
                {
                    ScalarValue value;
                    error = CNSDataGetScalarValue(fields[i], &value);
                    ERROR_CHECK("CNSDataGetScalarValue", error);
                    if (type == "string")
                    {
                        doCallbacks(dtype, value.c, 1, i);
                        error = CNSFreeMemory(value.c);
                        ERROR_CHECK("CNSFreeMemory", error);
                        m_bytes_read += strlen(value.c);
                    }
                    else
                    {
                        doCallbacks(dtype, &value, 1, i);
                        m_bytes_read += getCNSSize(dtype);
                    }
                }
            }
            for(int i=0; i<nfields; ++i)
            {
                error = CNSDataDiscard(fields[i]);
                ERROR_CHECK("CNSDataDiscard", error);
            }
            delete[] fields;
            ++m_items_read;
        }
        else if (type.find("array") != std::string::npos)
        {
            void* array = NULL;
            size_t num_elements;
            error = CNSReadArray(endpointID, CNSWaitForever, &array, &num_elements);
            ERROR_CHECK("CNSReadArray", error);
            epicsTimeGetCurrent(&epicsTS);
            doCallbacks(endpointType, array, num_elements);
            error = CNSFreeMemory(array);
            ERROR_CHECK("CNSFreeMemory", error);
            m_bytes_read += num_elements * getCNSSize(endpointType);
            ++m_items_read;
        }
        else
        {
            ScalarValue value;
            error = CNSReadScalar(endpointID, CNSWaitForever, &value);
            ERROR_CHECK("CNSReadScalar", error);
            epicsTimeGetCurrent(&epicsTS);
            if (type == "string")
            {
                doCallbacks(endpointType, value.c, 1);
                error = CNSFreeMemory(value.c);
                ERROR_CHECK("CNSFreeMemory", error);
                m_bytes_read += strlen(value.c);
            }
            else
            {
                doCallbacks(endpointType, &value, 1);
                m_bytes_read += getCNSSize(endpointType);
            }
            ++m_items_read;
        }
    }

    static void connectC(void * arg)
    {
            NsEndpoint* ep = (NsEndpoint*)arg;
            if (ep->access & NsEndpoint::Read)
            {
   			    ep->connectAndRead();
            }
            else
            {
                ep->monitorAndReconnect();
            }
    }
};


//template <CNSType T>
//void NetStreamsInterface::netStreamReader(const std::string& param)
//{
//	NsItem* item = m_params[param];
 //   CNS2C<T>::ctype val;
//	int status = CNSReadScalar(item->endpointID, CNSWaitForever, &val);
//	ERROR_CHECK("CNSReadScalar", status);
//					epicsTimeGetCurrent(&(item->epicsTS));
//					updateParamValue(item->id, ival, &(item->epicsTS), true);
 //   CNS2C<T>::free(val);
    

template<typename T>
void NetStreamsInterface::updateParamValue(int param_index, T val, epicsTimeStamp* epicsTS, bool do_asyn_param_callbacks)
{
	const char *paramName = NULL;
	m_driver->getParamName(param_index, &paramName);
	m_driver->lock();
	m_driver->setTimeStamp(epicsTS);
    m_params[paramName]->epicsTS = *epicsTS;
	if (m_params[paramName]->type == "float64" || m_params[paramName]->type == "ftimestamp")
	{
	    m_driver->setDoubleParam(param_index, convertToScalar<double>(val));
	}
	else if (m_params[paramName]->type == "int32" || m_params[paramName]->type == "boolean")
	{
	    m_driver->setIntegerParam(param_index, convertToScalar<int>(val));
	}
	else if (m_params[paramName]->type == "string" || m_params[paramName]->type == "timestamp")
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

template<typename T,typename U>
void NetStreamsInterface::updateParamArrayValueImpl(int param_index, T* val, size_t nElements)
{
	const char *paramName = NULL;
	m_driver->getParamName(param_index, &paramName);
	std::vector<char>& array_data =  m_params[paramName]->array_data;
	U* eval = convertToPtr<U>(val);
	if (eval != 0)
	{
		array_data.resize(nElements * sizeof(T));
		memcpy(&(array_data[0]), eval, nElements * sizeof(T));
		(m_driver->*C2CNS<U>::asyn_callback)(reinterpret_cast<U*>(&(array_data[0])), nElements, param_index, 0);
	}
	else
	{
		std::cerr << "updateParamArrayValue: cannot update param \"" << paramName << "\": shared variable data type incompatible \"" << C2CNS<T>::desc << "\"" << std::endl;
	}
}

// labview timestamp is seconds since 01-01-1904 00:00:00
// epics timestamp epoch is seconds since 01-01-1990 00:00:00
static void convertLabviewTimeToEpicsTime(const uint64_t* lv_time, epicsTimeStamp* epicsTS)
{
    static const uint64_t epoch_diff = 2713996800u; // seconds from 01-01-1904 to 01-01-1990
    static const uint64_t to_nsec = std::numeric_limits<uint64_t>::max() / 1000000000u;
    epicsTS->secPastEpoch = lv_time[0] - epoch_diff;
    epicsTS->nsec = lv_time[1] / to_nsec;
}

template<typename T>
void NetStreamsInterface::updateParamArrayValue(int param_index, T* val, size_t nElements, epicsTimeStamp* epicsTS, bool do_asyn_param_callbacks, bool with_ts)
{
	const char *paramName = NULL;
	m_driver->getParamName(param_index, &paramName);
    if (with_ts) // first 128bits of data are timestamp
    {
        size_t n_ts_elem = 16 / sizeof(T);
        const uint64_t* time_data = reinterpret_cast<const uint64_t*>(val);
        if (nElements > n_ts_elem)
        {
            epicsTimeStamp epicsTSv;
            convertLabviewTimeToEpicsTime(time_data, &epicsTSv);
            updateParamArrayValue(param_index, val + n_ts_elem, nElements - n_ts_elem, &epicsTSv, do_asyn_param_callbacks, false);
        }
        else
        {
            std::cerr << "updateParamArrayValue: param \"" << paramName << "\" not enough elements fro timestamp" << std::endl;
        }
        return;
    }
	m_driver->lock();
	m_driver->setTimeStamp(epicsTS);
	m_params[paramName]->epicsTS = *epicsTS;
	if (m_params[paramName]->type == "float64array")
	{
		updateParamArrayValueImpl<T,epicsFloat64>(param_index, val, nElements);
	}
	else if (m_params[paramName]->type == "float32array")
	{
		updateParamArrayValueImpl<T,epicsFloat32>(param_index, val, nElements);
	}
	else if (m_params[paramName]->type == "int32array")
	{
		updateParamArrayValueImpl<T,epicsInt32>(param_index, val, nElements);
	}
	else if (m_params[paramName]->type == "int16array")
	{
		updateParamArrayValueImpl<T,epicsInt16>(param_index, val, nElements);
	}
	else if (m_params[paramName]->type == "int8array")
	{
		updateParamArrayValueImpl<T,epicsInt8>(param_index, val, nElements);
	}
	else if (m_params[paramName]->type == "timestamp" || m_params[paramName]->type == "ftimestamp") // this is an array of two uint64 elements 
	{
        if ( nElements == 2 && sizeof(T) == sizeof(uint64_t) )
        {
            uint64_t* time_data = reinterpret_cast<uint64_t*>(val);
            epicsTimeStamp epicsTSv;
            convertLabviewTimeToEpicsTime(time_data, &epicsTSv);
	        // we do not need to call m_driver->setTimeStamp(&epicsTSv) etc as this is done in updateParamValue
            if (m_params[paramName]->type == "timestamp")
            {                
                char time_buffer[40]; // max size of epics simple string type
                epicsTimeToStrftime(time_buffer, sizeof(time_buffer), "%Y-%m-%dT%H:%M:%S.%06f", &epicsTSv);
                updateParamValue(param_index, time_buffer, &epicsTSv, do_asyn_param_callbacks);
            }
            else
            {
                double dval = epicsTSv.secPastEpoch + epicsTSv.nsec / 1e9;
                updateParamValue(param_index, dval, &epicsTSv, do_asyn_param_callbacks);
            }
        }
        else
        {
	        std::cerr << "updateParamArrayValue: timestamp param \"" << paramName << "\" not given UInt64[2] array" << std::endl;
        }
	}
	else
	{
	    std::cerr << "updateParamArrayValue: unknown type \"" << m_params[paramName]->type << "\" for param \"" << paramName << "\"" << std::endl;
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
	getParams();
	for(params_t::iterator it=m_params.begin(); it != m_params.end(); ++it)
	{
		NsItem* item = it->second;
        if (item->id != -1)
		{
			continue; // already initialised
		}
		if (item->type == "float64" || item->type == "ftimestamp")
		{
			item->nstype = CNSTypeDouble;
			m_driver->createParam(it->first.c_str(), asynParamFloat64, &(item->id));
		}
		else if (item->type == "int32" || item->type == "boolean")
		{
			item->nstype = CNSTypeInt32;
			m_driver->createParam(it->first.c_str(), asynParamInt32, &(item->id));
		}
		else if (item->type == "string" || item->type == "timestamp")
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
		std::string delay_ms_s = node.node().attribute("delay_ms").value();
		std::string buffer_items_s = node.node().attribute("buffer_items").value();
		std::string buffer_alloc_s = node.node().attribute("buffer_alloc").value();
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
        int delay_ms = (delay_ms_s.size() > 0 ? atoi(delay_ms_s.c_str()) : 0);
        size_t buffer_items = (buffer_items_s.size() > 0 ? atoi(buffer_items_s.c_str()) : 100);
        size_t buffer_alloc = (buffer_alloc_s.size() > 0 ? atoi(buffer_alloc_s.c_str()) : 0);
		NsEndpoint* nsep = new NsEndpoint(type, access_mode, URL, otherURL, delay_ms, buffer_items, buffer_alloc);
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
	for (pugi::xpath_node_set::const_iterator it = params.begin(); it != params.end(); ++it)
	{
		pugi::xpath_node node = *it;	
		std::string name = node.node().attribute("name").value();
		std::string type = node.node().attribute("type").value();
		std::string endpoint = node.node().attribute("endpoint").value();
		std::string field_s = node.node().attribute("field").value();	
		std::string ts_param = node.node().attribute("ts_param").value();	
		std::string with_ts_s = node.node().attribute("with_ts").value();	
		if (field_s.size() == 0)
		{
			field = -1;
		}
		else
		{
			field = atoi(field_s.c_str());
		}
        bool with_ts = false;
        if (with_ts_s == "true")
        {
            with_ts = true;
        }
        NsItem* ns = new NsItem(this, endpoint, type, field, ts_param, with_ts);
		m_params[name] = ns;
		if (m_endpoints.find(endpoint) != m_endpoints.end())
		{
			m_endpoints[endpoint]->m_callbacks.push_back(ns);
		}
		else
		{
			std::cerr << "Endpoint " << endpoint << " does not exist for parameter " << name << std::endl;
		}
	}	
}

template <>
void NetStreamsInterface::setValue(const char* param, const std::string& value)
{
	NsItem* item = m_params[param];
	if (C2CNS<char*>::nstype != item->nstype)
	{
        throw std::runtime_error(std::string("setValue: incorrect type for param \"") + param + "\" " + C2CNS<char*>::desc + " != " + item->type);
	}		
	if (item->field != -1)
	{
        throw std::runtime_error(std::string("setValue: unable to update struct variable via param \"") + param + "\"");
	}
	if (m_endpoints[item->endpoint]->access & NsEndpoint::Write)
	{
	    int status = CNSWriteScalar(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, value.c_str());
		ERROR_CHECK("CNSWriteScalar", status);
		status = CNSFlush(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, CNSFlushAllItemsAvailForRead);
		ERROR_CHECK("CNSFlush", status);
		epicsTimeGetCurrent(&(item->epicsTS));
	}
	else
	{
        throw std::runtime_error(std::string("setValue: param \"")  + param + "\" does not define a writer for \"" + item->endpoint + "\"");
	}
}

template <typename T>
void NetStreamsInterface::setValue(const char* param, const T& value)
{
	NsItem* item = m_params[param];
	if (C2CNS<T>::nstype != item->nstype)
	{
        throw std::runtime_error(std::string("setValue: incorrect type for param \"") + param + "\" " + C2CNS<T>::desc + " != " + item->type);
	}		
	if (item->field != -1)
	{
        throw std::runtime_error(std::string("setValue: unable to update struct variable via param \"") + param + "\"");
	}
	if (m_endpoints[item->endpoint]->access & NsEndpoint::Write)
	{
	    int status = CNSWriteScalar(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, value);
		ERROR_CHECK("CNSWriteScalar", status);
		status = CNSFlush(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, CNSFlushAllItemsAvailForRead);
		ERROR_CHECK("CNSFlush", status);
		epicsTimeGetCurrent(&(item->epicsTS));
	}
	else
	{
        throw std::runtime_error(std::string("setValue: param \"")  + param + "\" does not define a writer for \"" + item->endpoint + "\"");
	}
}

template <typename T>
void NetStreamsInterface::setArrayValue(const char* param, const T* value, size_t nElements)
{
	size_t dimensions[1] = { nElements };
	NsItem* item = m_params[param];
	if (C2CNS<T>::nstype != item->nstype)
	{
        throw std::runtime_error(std::string("setArrayValue: incorrect type for param \"") + param + "\" " + C2CNS<T>::desc + " != " + item->type);
	}		
	if (item->field != -1)
	{
        throw std::runtime_error(std::string("setArrayValue: unable to update struct variable via param \"") + param + "\"");
	}
	if (m_endpoints[item->endpoint]->access & NsEndpoint::Write)
	{
	    int status = CNSWriteArray(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, value, nElements);
		ERROR_CHECK("CNSWriteArray", status);
		status = CNSFlush(m_endpoints[item->endpoint]->endpointID, CNSWaitForever, CNSFlushAllItemsAvailForRead);
		ERROR_CHECK("CNSFlush", status);
		epicsTimeGetCurrent(&(item->epicsTS));
	}
	else
	{
        throw std::runtime_error(std::string("setArrayValue: param \"")  + param + "\" does not define a writer for \"" + item->endpoint + "\"");
	}
}

    epicsTimeStamp* NetStreamsInterface::getLinkedTimestamp(int param_id, epicsTimeStamp* es)
    {
	    const char *paramName = NULL;
        m_driver->getParamName(param_id, &paramName);
   	    const std::string& ts_param = m_params[paramName]->ts_param;
	    if (ts_param.size() > 0)
	    {
		    es = &(m_params[ts_param]->epicsTS);
	    }
        return es;
    }

/// Helper for EPICS driver report function
void NetStreamsInterface::report(FILE* fp, int details)
{
	fprintf(fp, "XML ConfigFile: \"%s\"\n", m_configFile.c_str());
	fprintf(fp, "XML ConfigFile section: \"%s\"\n", m_configSection.c_str());
	fprintf(fp, "NetStreamsConfigure() Options: %d\n", m_options);
	for(endpoints_t::iterator it=m_endpoints.begin(); it != m_endpoints.end(); ++it)
    {
    	NsEndpoint* ep = it->second;
        ep->report(it->first, fp);
    }
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
