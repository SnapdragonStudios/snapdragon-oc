cmake_minimum_required(VERSION 3.4.1)

# source
aux_source_directory(./src SRC)

# header
file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp)
source_group("Include" FILES ${CURRENT_HEADERS})

if (SDOC_IOS)
    # Temporary solution: To print the log, mark the source files as OBJCXX, which is a mixed type contains both C++ and Obj-C
    set_source_files_properties(${SRC} PROPERTIES LANGUAGE OBJCXX)
endif()

# library
add_library(RapidRasterizer STATIC ${SRC} ${CURRENT_HEADERS})

# link
target_link_libraries(RapidRasterizer Common)
