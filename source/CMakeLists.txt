# Notes:
# - make sure you are using a g++ cross compiler for arm, version 4.8 or above for c++11 compatibility
# - when cross compiling use the variable CMAKE_TOOLCHAIN_FILE
#       Ex.: -DCMAKE_TOOLCHAIN_FILE="E:/cosmos-source/projects/neutron1/source/cmake/toolchain_arm_win"
#       also note that CMAKE_SYSTEM_NAME is the target platform (ex: Linux) and CMAKE_HOST_SYSTEM_NAME is the
#       platform where the code is being compiled (Ex: Windows)

#--------------------------------------------------------------------
# user config variables
#--------------------------------------------------------------------

if(CROSS_TYPE)
    message("Compiling for ${CROSS_TYPE}")
else()
    message("Compiling for x86")
    SET(CROSS_TYPE "x86")
endif()
#SET(COSMOS_SOURCE_CORE "C:/cosmos/src/core/")
#include(${COSMOS_SOURCE}/core/cmake/toolchain-win-to-arm-sysgcc.cmake)


# define the project names
SET(PROJECT_NAME demo)

IF (${CMAKE_BUILD_TYPE} MATCHES "Debug")
MESSAGE("Building Debug version")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline-functions")
if (${CROSS_TYPE} MATCHES "arm")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -gdwarf-2")
endif()
ENDIF()

IF (${CMAKE_BUILD_TYPE} MATCHES "Release")
MESSAGE("Building Release version")
ENDIF()

# options: all, none, ...
# SET(ADD_EXECUTABLES "all")

# define folder where your programs are going to be installed
#SET(INSTALL_DIR                run)

#--------------------------------------------------------------------
# automatic config section, you should not have to edit further
#--------------------------------------------------------------------
# comment
#SET(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
cmake_minimum_required(VERSION 2.8.12)
project(${PROJECT_NAME})

Option(BUILD_GPROF "Build for Profiling" OFF)

IF (BUILD_GPROF)
MESSAGE("Building for gprof Profiler")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -Wtype-limits -Wall -pedantic -std=c++11 -U__STRICT_ANSI__ -Wno-invalid-offsetof")
ELSE ()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wall -pedantic -std=c++11 -U__STRICT_ANSI__ -Wno-invalid-offsetof")
ENDIF ()


include(cmake/macros.cmake)

# -----------------------------------------------
set(COSMOS_SOURCE ${PROJECT_SOURCE_DIR}/../../..)
message ("COSMOS_SOURCE: ${COSMOS_SOURCE}")

# compile on Linux host
IF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")

MESSAGE("Compiling on Linux for ${CROSS_TYPE}")
SET( "${COSMOS_SOURCE}/core/")

# Linux specific code
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#Check for ARM Build
IF (${CROSS_TYPE} MATCHES "arm" OR ${CROSS_TYPE} MATCHES "iobc")

# ARM Build Detected

# Set installation directory
IF("$ENV{COSMOS}" STREQUAL "")
SET(COSMOS "~/cosmos/${CROSS_TYPE}")
message("ARM Default COSMOS: ${COSMOS}")
ELSE()
SET(COSMOS "$ENV{COSMOS}/${CROSS_TYPE}")
message("ARM Environment COSMOS: ${COSMOS}")
ENDIF("$ENV{COSMOS}" STREQUAL "")

# Set Toolchain File
SET(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/build/toolchain_${CROSS_TYPE}")

# Add preprocessor definition to help source files determine code to use for ARMv7 specific implementation
add_definitions(-DCROSS_TYPE_arm) #20150825JC: Added definition

# Add include directories for locally compiled arm openssl
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/arm/include)

ELSE()

# Non-ARM Build Detected

IF("$ENV{COSMOS}" STREQUAL "")
SET(COSMOS "~/cosmos")
ELSE()
SET(COSMOS "$ENV{COSMOS}")
ENDIF("$ENV{COSMOS}" STREQUAL "")

ENDIF()

ENDIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")

# -----------------------------------------------
# compile on Mac host
IF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")

MESSAGE("Compiling on Mac")

# Mac OS X specific code
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
IF("$ENV{COSMOS}" STREQUAL "")
SET(COSMOS "/Applications/cosmos/macos")
ELSE()
SET(COSMOS "$ENV{COSMOS}/macos")
ENDIF("$ENV{COSMOS}" STREQUAL "")

SET(COSMOS_SOURCE_CORE "${COSMOS_SOURCE}/core/")
ENDIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")


# -----------------------------------------------
# compile on Windows host
IF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")

MESSAGE("---------------------")
MESSAGE("Compiling on Windows")

# Windows 7 specific code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#   Option(${CROSS_TYPE} MATCHES "arm" "Build for ARM architecture" OFF)
IF (${CROSS_TYPE} MATCHES "arm")
SET(COSMOS "c:/cosmos/arm")
IF("$ENV{COSMOS}" STREQUAL "")
#            SET(COSMOS_SOURCE "c:/cosmos/src")
ELSE()
#            SET(COSMOS_SOURCE "$ENV{COSMOS}/src")
ENDIF("$ENV{COSMOS}" STREQUAL "")

SET(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/cmake/toolchain_arm_win")

# Add include directories for locally compiled arm openssl
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/arm/include)

ELSE()
set(WINDOWS_LIBS  wsock32 winmm ws2_32 iphlpapi)
IF("$ENV{COSMOS}" STREQUAL "")
SET(COSMOS "c:/cosmos")
ELSE()
SET(COSMOS "$ENV{COSMOS}")
ENDIF("$ENV{COSMOS}" STREQUAL "")
ENDIF()


# TODO: implement the option to compile the code using cosmos-source directly

# SET(COSMOS_SOURCE_CORE "e:/cosmos-source/core/")
# add_subdirectory(${COSMOS_SOURCE}/core cosmos/core)

SET(COSMOS_SOURCE_CORE "${COSMOS_SOURCE}/core/")

# Convert "/" to "\" IF we are in Windows
#    string(REPLACE "\\" "/" COSMOS_SOURCE ${COSMOS_SOURCE})

ENDIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")


# -----------------------------------------------

set(USE_COSMOS_FROM "SOURCE")
include(${COSMOS_SOURCE}/core/cmake/use_cosmos_from_source.cmake)


# let's use the cosmos/core libraries
# localzlib myjpeg
set(COSMOS_LIBS ${WINDOWS_LIBS})

message("COSMOS: ${COSMOS}")

set(CMAKE_INSTALL_PREFIX "${COSMOS}/${PROJECT_NAME}" CACHE PATH "default install path" FORCE )
message("Install: ${CMAKE_INSTALL_PREFIX}")

#include_directories(${COSMOS}/include)
include_directories(${COSMOS_SOURCE}/core/libraries)
#include_directories(${COSMOS}/hsfl_libs/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries)

MESSAGE("Include: ${COSMOS}/include")

add_subdirectory(${PROJECT_SOURCE_DIR}/libraries)

# -----------------------------------------------

# Add external modules


# Note that the modules folder must be cloned/added in manually
include_directories(${COSMOS_SOURCE}/modules/devices/cubespace/libraries)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/cubespace/libraries     ${CMAKE_BINARY_DIR}/libraries/cubespace/)

#include_directories(${COSMOS_SOURCE}/modules/devices/eps/clydespace/libraries)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/eps/clydespace/libraries     ${CMAKE_BINARY_DIR}/libraries/eps/clydespace/libraries)

#include_directories(${COSMOS_SOURCE}/modules/devices/scpi/general/libraries)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/scpi/general/libraries     ${CMAKE_BINARY_DIR}/libraries/scpi/general/libraries)

#include_directories(${COSMOS_SOURCE}/modules/devices/isis/libraries/cob)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/isis/libraries/cob     ${CMAKE_BINARY_DIR}/libraries/isis/cob)

#include_directories(${COSMOS_SOURCE}/modules/devices/isis/libraries/antenna)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/isis/libraries/antenna     ${CMAKE_BINARY_DIR}/libraries/isis/antenna)

#include_directories(${COSMOS_SOURCE}/modules/devices/isis/libraries/trxvu)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/isis/libraries/trxvu     ${CMAKE_BINARY_DIR}/libraries/isis/trxvu)

include_directories(${COSMOS_SOURCE}/modules/devices/gps/oemv/libraries)
#add_subdirectory(${COSMOS_SOURCE}/modules/devices/gps/oemv/libraries     ${CMAKE_BINARY_DIR}/libraries/gps/oemv)

# add the Programs
add_subdirectory(${PROJECT_SOURCE_DIR}/programs)

# 20150826JC: Added to confirm flags
MESSAGE("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
MESSAGE("CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
