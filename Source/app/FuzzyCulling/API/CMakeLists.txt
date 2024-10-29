cmake_minimum_required(VERSION 3.4.1)

# source
aux_source_directory(./private SRC)

# header
file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp)
source_group("Include" FILES ${CURRENT_HEADERS})

if (SDOC_IOS)
    # Temporary solution: To print the log, mark the source files as OBJCXX, which is a mixed type contains both C++ and Obj-C
    set_source_files_properties(${SRC} PROPERTIES LANGUAGE OBJCXX)
endif()

# library
if (SDOC_IOS)
    add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS})
elseif (SDOC_WIN AND SDOC_STATIC)
	add_library(Commonlib OBJECT IMPORTED)
	if(SDOC_DEBUG)
		set_property(TARGET Commonlib PROPERTY IMPORTED_OBJECTS "$/../../Common/Debug/Common.lib")
		add_library(RapidRasterizerLib OBJECT IMPORTED)
		set_property(TARGET RapidRasterizerLib PROPERTY IMPORTED_OBJECTS "$/../../Util/RapidRasterizer/Debug/RapidRasterizer.lib")
	else()
		set_property(TARGET Commonlib PROPERTY IMPORTED_OBJECTS "$/../../Build/Common/Release/Common.lib")
		add_library(RapidRasterizerLib OBJECT IMPORTED)
		set_property(TARGET RapidRasterizerLib PROPERTY IMPORTED_OBJECTS "$/../../Build/Util/RapidRasterizer/Release/RapidRasterizer.lib")
	endif()

	if(SUPPORT_ALL_FEATURE)
		add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS}  $<TARGET_OBJECTS:Commonlib>  $<TARGET_OBJECTS:RapidRasterizerLib>  )
	else()		
		add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS}  $<TARGET_OBJECTS:Commonlib>  $<TARGET_OBJECTS:RapidRasterizerLib> )
	endif()
elseif (SDOC_OSX AND SDOC_STATIC)
	add_library(Commonlib OBJECT IMPORTED)
	if (SDOC_DEBUG)
		set_property(TARGET Commonlib PROPERTY IMPORTED_OBJECTS "$/../../Build/Common/Debug/libCommon.a")
		add_library(RapidRasterizerLib OBJECT IMPORTED)
		set_property(TARGET RapidRasterizerLib PROPERTY IMPORTED_OBJECTS "$/../../Build/Util/RapidRasterizer/Debug/libRapidRasterizer.a")
	else()
		set_property(TARGET Commonlib PROPERTY IMPORTED_OBJECTS "$/../../Build/Common/Release/libCommon.a")
		add_library(RapidRasterizerLib OBJECT IMPORTED)
		set_property(TARGET RapidRasterizerLib PROPERTY IMPORTED_OBJECTS "$/../../Build/Util/RapidRasterizer/Release/libRapidRasterizer.a")
	endif()
    add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS}  $<TARGET_OBJECTS:Commonlib>  $<TARGET_OBJECTS:RapidRasterizerLib> )
    #add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS})
elseif (SDOC_ANDROID_ARM AND SDOC_STATIC)
    add_library(Commonlib OBJECT IMPORTED)
   set_property(TARGET Commonlib PROPERTY IMPORTED_OBJECTS "$/../../Common/libCommon.a")
    add_library(RapidRasterizerLib OBJECT IMPORTED)
    set_property(TARGET RapidRasterizerLib PROPERTY IMPORTED_OBJECTS "$/../../Util/RapidRasterizer/libRapidRasterizer.a")
    add_library(SDOC.quic STATIC ${SRC} ${CURRENT_HEADERS}  $<TARGET_OBJECTS:Commonlib>  $<TARGET_OBJECTS:RapidRasterizerLib> )
	set_target_properties(SDOC.quic PROPERTIES LINK_FLAGS_RELEASE -s)
else()
    add_library(SDOC.quic SHARED ${SRC} ${CURRENT_HEADERS})
endif() 

# For iOS, build SDOC.quic as a dynamic framework
#if (SDOC_IOS)
#    set(PUBLIC_HEADER SDOCAPI.h)
#    set_target_properties(SDOC.quic PROPERTIES
#    FRAMEWORK TRUE
#    FRAMEWORK_VERSION A
#    MACOSX_FRAMEWORK_IDENTIFIER com.qualcomm.sdoc
#    # MACOSX_FRAMEWORK_INFO_PLIST Info.plist
#    # "current version" in semantic format in Mach-O binary file
#    VERSION 1.0.0
#    # "compatibility version" in semantic format in Mach-O binary file
#    SOVERSION 1.0.0
#    PUBLIC_HEADER "${PUBLIC_HEADER}"
#    # XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
#    )
#endif()

if (SDOC_WIN)
set_target_properties(SDOC.quic PROPERTIES PREFIX "lib")
endif()

# link
if (SDOC_ANDROID_ARM)
    target_link_libraries(SDOC.quic RapidRasterizer Common log)
elseif (SDOC_WIN AND !SDOC_STATIC)
    target_link_libraries(SDOC.quic RapidRasterizer Common)
else()
#   MAC IOS platforms
if(SUPPORT_ALL_FEATURE)
    target_link_libraries(SDOC.quic RapidRasterizer Common)
else()
    target_link_libraries(SDOC.quic RapidRasterizer Common)
endif()
endif()


  
# NO POST_BUILD ANY MORE
## Post-build for iOS
if (SDOC_IOS)
add_custom_command(TARGET SDOC.quic POST_BUILD
  COMMENT "Merge Static Libraries for iOS"
  COMMAND libtool -static -o libSDOC.quic.a libSDOC.quic.a libCommon.a libRapidRasterizer.a
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  VERBATIM
)
add_custom_command(TARGET SDOC.quic POST_BUILD
  COMMENT "Move to QOC demo"
  COMMAND cp libSDOC.quic.a ../../../../../Demo/iOS/iQOC/QOC/lib/QOC.quic.framework/QOC.quic
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  VERBATIM
)
endif()
