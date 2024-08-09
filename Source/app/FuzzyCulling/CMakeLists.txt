cmake_minimum_required(VERSION 3.4.1)
project(SDOC-Lib C CXX)

# shadow build
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
endif()


if(APPLE)
    execute_process(COMMAND uname -m
        OUTPUT_VARIABLE CMAKE_OSX_ARCHITECTURES
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif(APPLE)


# Platform
# Show the current platform
message(STATUS "---->Target: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message(STATUS "CMAKE_OSX_ARCHITECTURES: ${CMAKE_OSX_ARCHITECTURES}")

# SDOC_NATIVE:
#     SDOC_WIN
#     SDOC_OSX
# SDOC_MOBILE:
#     SDOC_ANDROID
#     SDOC_IOS
set(SDOC_NATIVE 0)
set(SDOC_WIN 0)
set(SDOC_OSX 0)
set(SDOC_OSX_ARM 0)
set(SDOC_OSX_x86 0)
set(SDOC_MOBILE 0)
set(SDOC_IOS 0)
set(SDOC_ANDROID 0)




set(SDOC_LIB_EXPORT 0)

if((${CMAKE_SYSTEM_NAME} STREQUAL Darwin) )
    if( ${CMAKE_OSX_ARCHITECTURES} STREQUAL arm64)
    	message(STATUS "SDOC Apple ARM Native")
    	set(SDOC_NATIVE 1)
    	add_definitions(-DSDOC_NATIVE)
    	set(SDOC_LIB_EXPORT 1)
    	add_definitions(-DSDOC_LIB_EXPORT)
    	set(SDOC_OSX 1)
    	set(SDOC_OSX_ARM 1)
    	add_definitions(-DSDOC_OSX)  
   else()
      message(STATUS "SDOC X86 Native")
      set(SDOC_NATIVE 1)
      add_definitions(-DSDOC_NATIVE)
	
      set(SDOC_LIB_EXPORT 1)
      add_definitions(-DSDOC_LIB_EXPORT)
      set(SDOC_OSX 1)
    	set(SDOC_OSX_x86 1)
      add_definitions(-DSDOC_OSX)
   endif()   
elseif ((${CMAKE_SYSTEM_NAME} STREQUAL Windows) )
    message(STATUS "SDOC X86 Native")
    set(SDOC_NATIVE 1)
    add_definitions(-DSDOC_NATIVE)
	
    set(SDOC_LIB_EXPORT 1)
    add_definitions(-DSDOC_LIB_EXPORT)
    set(SDOC_WIN 1)

    add_definitions(-DSDOC_WIN)
elseif ((${CMAKE_SYSTEM_NAME} STREQUAL Android) OR (${CMAKE_SYSTEM_NAME} STREQUAL iOS))
    message(STATUS "SDOC Mobile")
    set(SDOC_MOBILE 1)
    add_definitions(-DSDOC_MOBILE)
	
    set(SDOC_LIB_EXPORT 1)
    add_definitions(-DSDOC_LIB_EXPORT)

    if (${CMAKE_SYSTEM_NAME} STREQUAL Android)
        set(SDOC_ANDROID 1)
        add_definitions(-DSDOC_ANDROID)
    else()
        set(SDOC_IOS 1)
        add_definitions(-DSDOC_IOS)
    endif()
endif()


set(SDOC_DevelopMode 0)
##########################################################################################
################revert below setting for windows development
if (SDOC_DevelopMode)
	#disable below two macro for release SDOC dynamic library dll/dsym
	set(SDOC_NATIVE_DEBUG 1)
	add_definitions(-DSDOC_NATIVE_DEBUG)
	set(SUPPORT_ALL_FEATURE 1)
	add_definitions(-DSUPPORT_ALL_FEATURE)
else()
endif()
##########################################################################################
##########################################################################################
#set(SDOC_STRESS_TEST 1)
#add_definitions(-DSDOC_STRESS_TEST)
##########################################################################################
##########################################################################################
##########################################################################################



##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
#Unity Want Static library
set(SDOC_UNITY_STATIC 0) 
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################

set(SDOC_STATIC 0)

if (SDOC_IOS)
	set(SDOC_STATIC 1)
	add_definitions(-DSDOC_STATIC)
endif()

if(SDOC_UNITY_STATIC) #build static library for Win/Mac
	set(SDOC_STATIC 1)
	add_definitions(-DSDOC_STATIC)
endif()


#set(SDOC_DEBUG 1)
#add_definitions(-DSDOC_DEBUG)


# CMAKE_CXX_FLAGS
if (SDOC_IOS)
    # for iOS, set OBJCXX flags
    set(CMAKE_OBJCXX_FLAGS "${CMAKE_OBJCXX_FLAGS} -std=c++14")
    set(CMAKE_OBJCXX_FLAGS_RELEASE "${CMAKE_OBJCXX_FLAGS} -Os -fvisibility=hidden")
elseif (SDOC_ANDROID AND SDOC_STATIC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g0")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Os -fvisibility=hidden -g0")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Os -fvisibility=hidden")
endif()

# for OSX, set "-march" to native in order to support FMA
if (SDOC_OSX_x86)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma")
    #set(CMAKE_XCODE_ATTRIBUTE_MACOSX_DEPLOYMENT_TARGET "10.13")
endif()

# Put all the libraries to the /lib folder, for post-build purpose
if (SDOC_IOS)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
endif()

# MSVC Output
if (SDOC_WIN)
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(./)

# subdirectory
add_subdirectory(API)
add_subdirectory(Common)
add_subdirectory(Util)

# Add Test folder for Desktop
if (SDOC_NATIVE OR SDOC_OSX )
    add_subdirectory(Test)
endif()
