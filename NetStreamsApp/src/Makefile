TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

USR_INCLUDES += -ICVI/include
#USR_CXXFLAGS_Linux += -std=c++11
USR_CXXFLAGS_Linux += -std=c++0x

ifeq ($(STATIC_BUILD),NO)
USR_LDFLAGS_WIN32 += /NODEFAULTLIB:LIBCMT.LIB /NODEFAULTLIB:LIBCMTD.LIB
endif

LIBRARY_IOC += NetStreams

ifeq (WIN32,$(OS_CLASS))
NICVIDIR=$(subst /,\,$(ICPBINARYDIR)/CVI2013)
else
#NICVIDIR=$(ICPBINARYDIR)/CVI2013
NICVIDIR=/usr/local/natinst/cvi2013
endif

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install NetStreams.dbd into <top>/dbd
DBD += NetStreams.dbd

# specify all source files to be compiled and added to the library
NetStreams_SRCS += convertToString.cpp NetStreamsDriver.cpp 
NetStreams_SRCS += pugixml.cpp cnsconvert.cpp NetStreamsInterface.cpp

NetStreams_LIBS += asyn
NetStreams_LIBS += $(EPICS_BASE_IOC_LIBS)

ifeq (WIN32,$(OS_CLASS))
ifneq ($(findstring windows,$(EPICS_HOST_ARCH)),)
CVILIB = CVI/extlib/msvc64
else
CVILIB = CVI/extlib/msvc
endif
else
CVILIB = /usr/local/lib
endif

NetStreams_SYS_LIBS_WIN32 += $(CVILIB)/cvinetstreams $(CVILIB)/cvisupp $(CVILIB)/cvirt 
NetStreams_SYS_LIBS_WIN32 += user32

NetStreams_SYS_LIBS_Linux += ninetstreams

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# we ned to make a copy due to spaces in the absolute path
../cnsconvert.cpp : CVI
CVI :
	-mkdir CVI
ifeq (WIN32,$(OS_CLASS))
	xcopy /D /I /E /Q /Y /R $(NICVIDIR)\include $@\include
	xcopy /D /I /E /Q /Y /R $(NICVIDIR)\extlib $@\extlib
endif
ifeq (Linux,$(OS_CLASS))
	cp -r $(NICVIDIR)/include $@
#	cp -r $(NICVIDIR)/extlib $@
endif

