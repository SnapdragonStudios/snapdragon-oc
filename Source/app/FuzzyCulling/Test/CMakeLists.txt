cmake_minimum_required(VERSION 3.4.1)

if (SDOC_OSX_ARM)
elseif (SDOC_OSX)
    set(CMAKE_OSX_ARCHITECTURES x86_64)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma")
    #set(CMAKE_XCODE_ATTRIBUTE_MACOSX_DEPLOYMENT_TARGET "10.13")
endif()


aux_source_directory(./ SRC)
# header
file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp)
source_group("Include" FILES ${CURRENT_HEADERS})

add_executable(SDOCReplay ${SRC} ${CURRENT_HEADERS})
target_link_libraries(SDOCReplay SDOC.quic)
	
