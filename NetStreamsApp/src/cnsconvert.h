/*************************************************************************\ 
* Copyright (c) 2013 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

/// @file cnsconvert.h Header file for network shared variable to EPICS type convertion routines.
/// @author Freddie Akeroyd, STFC ISIS Facility, GB

#ifndef CNSCONVERT_H
#define CNSCONVERT_H

#include <type_traits>

// CNS to C

/// Provide the underlying C data type \a ctype for a given network shared variable type
template<CNSType type>
struct CNS2C
{
    typedef int ctype;   ///< an instance of the underlying type
	static const char* desc;   ///< description of type
	static void free(ctype);    ///< function to free any memory associated with type
};

template<>
struct CNS2C<CNSTypeBool>
{
    typedef char ctype;    // char not bool as we need a 1 byte data type for C call and i'm not sure about "bool"
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeString>
{
    typedef char* ctype;
	static const char* desc;
	static void free(ctype val)
	{ 
	    if (val != 0) 
		{
		    CNSFreeMemory(val);
		}
	}
};

template<>
struct CNS2C<CNSTypeSingle>
{
    typedef float ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeDouble>
{
    typedef double ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeInt8>
{
    typedef char ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeUInt8>
{
    typedef unsigned char ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeInt16>
{
    typedef short ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeUInt16>
{
    typedef unsigned short ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeInt32>
{
    typedef int ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeUInt32>
{
    typedef unsigned int ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeInt64>
{
    typedef __int64 ctype;
	static const char* desc;
	static void free(ctype val) { }
};

template<>
struct CNS2C<CNSTypeUInt64>
{
    typedef unsigned __int64 ctype;
	static const char* desc;
	static void free(ctype val) { }
};

// C type to CNS

/// For a given C data type, provide the appropriate network shared variable type
template<typename T>
struct C2CNS
{
    enum { nstype = -1 };   // never used
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(T* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<bool>
{
    enum { nstype = CNSTypeBool };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt8* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<char*>
{
    enum { nstype = CNSTypeString };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(char** value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<float>
{
    enum { nstype = CNSTypeSingle };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsFloat32* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<double>
{
    enum { nstype = CNSTypeDouble };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsFloat64* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<char>
{
    enum { nstype = CNSTypeInt8 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt8* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<signed char>
{
    enum { nstype = CNSTypeInt8 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt8* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<unsigned char>
{
    enum { nstype = CNSTypeUInt8 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt8* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<short>
{
    enum { nstype = CNSTypeInt16 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt16* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<unsigned short>
{
    enum { nstype = CNSTypeUInt16 };
	static const char* desc;
	static asynStatus(asynPortDriver::*asyn_callback)(epicsInt16* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<int>
{
    enum { nstype = CNSTypeInt32 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt32* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<unsigned int>
{
    enum { nstype = CNSTypeUInt32 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(epicsInt32* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<__int64>
{
    enum { nstype = CNSTypeInt64 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(__int64* value, size_t nElements, int reason, int addr);
};

template<>
struct C2CNS<unsigned __int64>
{
    enum { nstype = CNSTypeUInt64 };
	static const char* desc;
	static asynStatus (asynPortDriver::*asyn_callback)(__int64* value, size_t nElements, int reason, int addr);
};

// type convertions - used when we set asyn parameters. We want to cast basic type -> basic type and pointer -> pointer, but ignore
// everything else which shouldn't get called anyway.
// e.g.    convertToScalar<double>(value)

/// convert one type to another
template<typename T, typename U>
static T convertToScalar(U val)
{
    return static_cast<T>(val);
}

template<typename T, typename U>
static T convertToScalar(U* val)
{
    throw std::runtime_error("convertToScalar: illegal cast of pointer to simple type");
//    return static_cast<T>(0);
}

template<typename T, typename U>
static T* convertToPtr(U val)
{
    throw std::runtime_error("convertToPtr: illegal cast of simple type to pointer");
//   return static_cast<T*>(0);
}

/// default case handles already signed types i.e. <T,false>
template <typename T, bool is_unsigned>
struct MakeSignedImpl
{
    typedef T type;
};

/// specialisation of MakeSignedImpl for unsigned types i.e. <T,true>
template <typename T>
struct MakeSignedImpl<T,true>
{
    typedef typename std::make_signed<T>::type type;
};

/// like std::make_signed but also handles bool,float etc. types by passing them through rather than producing an error
template <typename T>
struct MakeSigned
{
    typedef typename MakeSignedImpl< T, std::is_unsigned<T>::value >::type type;
};

/// Types that differ only in sign are considered castable as epics asyn doesn't have unsigned data types for arrays
template<typename T, typename U>
static T* convertToPtr(U* val)
{
    if ( std::is_same< typename MakeSigned< typename std::remove_cv<T>::type >::type, typename MakeSigned< typename std::remove_cv<U>::type >::type >::value )
	{
        return reinterpret_cast<T*>(val);
	}
	else
	{
        return 0;
	}
}

#endif /* CNSCONVERT_H */
