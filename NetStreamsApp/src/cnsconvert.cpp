/*************************************************************************\ 
* Copyright (c) 2018 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

/// @file cnsconvert.cpp Network shared variable type convertion routine.
/// @author Freddie Akeroyd, STFC ISIS Facility, GB

#include <stdexcept>

#include <cvirte.h>		
#include <userint.h>
#include <cvinetstreams.h>

#include <asynPortDriver.h>

#include "cnsconvert.h"

const char* CNS2C<CNSTypeBool>::desc = "bool";
const char* CNS2C<CNSTypeString>::desc = "char*";
const char* CNS2C<CNSTypeSingle>::desc = "float";
const char* CNS2C<CNSTypeDouble>::desc = "double";
const char* CNS2C<CNSTypeInt8>::desc = "char";
const char* CNS2C<CNSTypeUInt8>::desc = "unsigned char";
const char* CNS2C<CNSTypeInt16>::desc = "short";
const char* CNS2C<CNSTypeUInt16>::desc = "unsigned short";
const char* CNS2C<CNSTypeInt32>::desc = "int";
const char* CNS2C<CNSTypeUInt32>::desc = "unsigned int";
const char* CNS2C<CNSTypeInt64>::desc = "__int64";
const char* CNS2C<CNSTypeUInt64>::desc = "unsigned __int64";

const char* C2CNS<bool>::desc = "CNSTypeBool";
const char* C2CNS<char*>::desc = "CNSTypeString";
const char* C2CNS<float>::desc = "CNSTypeSingle";
const char* C2CNS<double>::desc = "CNSTypeDouble";
const char* C2CNS<char>::desc = "CNSTypeInt8";
const char* C2CNS<unsigned char>::desc = "CNSTypeUInt8";
const char* C2CNS<short>::desc = "CNSTypeInt16";
const char* C2CNS<unsigned short>::desc = "CNSTypeUInt16";
const char* C2CNS<int>::desc = "CNSTypeInt32";
const char* C2CNS<unsigned int>::desc = "CNSTypeUInt32";
const char* C2CNS<__int64>::desc = "CNSTypeInt64";
const char* C2CNS<unsigned __int64>::desc = "CNSTypeUInt64";

// asyn callbacks take signed types only
asynStatus (asynPortDriver::*C2CNS<double>::asyn_callback)(double* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksFloat64Array;
asynStatus (asynPortDriver::*C2CNS<float>::asyn_callback)(float* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksFloat32Array;
asynStatus (asynPortDriver::*C2CNS<int>::asyn_callback)(int* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt32Array;
asynStatus (asynPortDriver::*C2CNS<short>::asyn_callback)(short* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt16Array;
asynStatus (asynPortDriver::*C2CNS<char>::asyn_callback)(char* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt8Array;
asynStatus (asynPortDriver::*C2CNS<unsigned int>::asyn_callback)(int* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt32Array;
asynStatus (asynPortDriver::*C2CNS<unsigned short>::asyn_callback)(short* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt16Array;
asynStatus (asynPortDriver::*C2CNS<unsigned char>::asyn_callback)(char* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt8Array;
asynStatus (asynPortDriver::*C2CNS<bool>::asyn_callback)(char* value, size_t nElements, int reason, int addr) = &asynPortDriver::doCallbacksInt8Array;
