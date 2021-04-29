if (NOT DEFINED ENV{FFMPEG_PATH})
    message(FATAL_ERROR "Please define FFMPEG_PATH first!")
endif()

#FFMPEG Settings
set(FFMPEG_INCLUDE_DIRS ${FFMPEG_PATH}/include)
set(components avcodec avdevice avfilter avformat avutil swscale swresample)

foreach(component ${components})

    if (NOT DEFINED FFMPEG_PATH)
        find_path(${component}_include_dir lib${component}/${component}.h)
        if(${component}_include_dir)
            list(APPEND FFMPEG_INCLUDE_DIRS ${${component}_include_dir})
        endif()
    endif()

    set(FFMPEG_LIB_PATH PATHS ${FFMPEG_PATH} NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_PACKAGE_REGISTRY NO_CMAKE_FIND_ROOT_PATH)
    set(FFMPEG_LIB_SUFFIXES "lib")
    find_library(${component}_library NAMES ${component} NAMES_PER_DIR ${FFMPEG_LIB_PATH} PATH_SUFFIXES ${FFMPEG_LIB_SUFFIXES} NO_CMAKE_SYSTEM_PATH)
    if(${component}_library)
        list(APPEND FFMPEG_LIBRARIES ${${component}_library})
    endif()

endforeach()