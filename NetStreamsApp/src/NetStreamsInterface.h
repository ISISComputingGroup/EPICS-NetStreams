/*************************************************************************\ 
* Copyright (c) 2013 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

/// @file NetStreamsInterface.h header for #NetStreamsInterface class. 
/// @author Freddie Akeroyd, STFC ISIS Facility, GB

#ifndef NetStreams_INTERFACE_H
#define NetStreams_INTERFACE_H

#include <stdio.h>

#include <string>
#include <vector>
#include <map>
#include <list>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <iostream>

#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <macLib.h>

#include <cvirte.h>		
#include <userint.h>
#include <cvinetstreams.h>

#include "pugixml.hpp"

/// option argument in NetStreamsConfigure() of @link st.cmd @endlink not used at present
enum NetStreamsOptions { NSNothing = 0, NSSomething=1 };

class asynPortDriver;
struct NsItem;
struct NsEndpoint;

/// Manager class for the NetVar Interaction. Parses an @link netvarconfig.xml @endlink file and provides access to the 9variables described within. 
class NetStreamsInterface
{
public:
	NetStreamsInterface(const char* configSection, const char *configFile, int options);
	size_t nParams();
	~NetStreamsInterface() { }
	void updateValues();
	void createParams(asynPortDriver* driver);
	void report(FILE* fp, int details);
	template<typename T> void setValue(const char* param, const T& value);
	template<typename T> void setArrayValue(const char* param, const T* value, size_t nElements);
	template<typename T> void readArrayValue(const char* paramName, T* value, size_t nElements, size_t* nIn);

	template<typename T> void updateParamValue(int param_index, T val, epicsTimeStamp* epicsTS, bool do_asyn_param_callbacks);
    
private:
	std::string m_configSection;  ///< section of \a configFile to load information from
	std::string m_configFile;   
	int m_options; ///< the various #NetStreamsOptions currently in use
//	epicsMutex m_lock;
	asynPortDriver* m_driver;
	typedef std::map<std::string,NsItem*> params_t;
	params_t m_params;
	pugi::xml_document m_xmlconfig;
    MAC_HANDLE* m_mac_env;
	typedef std::map<std::string,NsEndpoint*> endpoints_t;
    endpoints_t m_endpoints;
	
    char* envExpand(const char *str);
	void getParams();
	void getEndpoints();    
	static void epicsExitFunc(void* arg);
	bool checkOption(NetStreamsOptions option) { return ( m_options & static_cast<int>(option) ) != 0; }
	void connectVars();
	template<typename T> void updateParamArrayValue(int param_index, T* val, size_t nElements, epicsTimeStamp* epicsTS);
	template<typename T,typename U> void updateParamArrayValueImpl(int param_index, T* val, size_t nElements);
};

#endif /* NETSTREAM_INTERFACE_H */
